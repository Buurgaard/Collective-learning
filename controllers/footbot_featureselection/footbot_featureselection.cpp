/* Include the controller definition */
#include "footbot_featureselection.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>




/****************************************/
/****************************************/

CFootBotFeatureSelection::CFootBotFeatureSelection() :
        m_pcWheels(NULL),
        m_pcProximity(NULL),
        m_pcGripper(NULL),
        m_pcCamera(NULL),
        m_pcRABA(NULL),
        m_pcRABS(NULL),
        m_pcDS(NULL),
        m_unCounter(0),
        m_cAlpha(10.0f),
        m_fDelta(0.5f),
        m_Epsilon(0),
        m_fWheelVelocity(2.5f),
        m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                                ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotFeatureSelection::Init(TConfigurationNode& t_node) {
    /*
     * Get sensor/actuator handles
     *
     * The passed string (ex. "differential_steering") corresponds to the
     * XML tag of the device whose handle we want to have. For a list of
     * allowed values, type at the command prompt:
     *
     * $ argos3 -q actuators
     *
     * to have a list of all the possible actuators, or
     *
     * $ argos3 -q sensors
     *
     * to have a list of all the possible sensors.
     *
     * NOTE: ARGoS creates and initializes actuators and sensors
     * internally, on the basis of the lists provided the configuration
     * file at the <controllers><footbot_diffusion><actuators> and
     * <controllers><footbot_diffusion><sensors> sections. If you forgot to
     * list a device in the XML and then you request it here, an error
     * occurs.
     */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcGripper   = GetActuator<CCI_FootBotGripperActuator      >("footbot_gripper"      );
    m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing");

    m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
    m_pcCamera    = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
    m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcDS        = GetSensor  <CCI_DifferentialSteeringSensor>("differential_steering");


    /*
     * Parse the configuration file
     *
     * The user defines this part. Here, the algorithm accepts three
     * parameters and it's nice to put them in the config file so we don't
     * have to recompile if we want to try other settings.
     */
    GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
    m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
    GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

    /* Switch the camera on */
    m_pcCamera->Enable();
    m_pcRABA->ClearData();
    m_pcRABA->SetData(0,0);

    GetNodeAttribute(t_node, "epsilon", m_Epsilon);

    state = EXPLORE; //explore
    srand((unsigned) time(NULL));


    // Index magic
    std::fstream indexFileIn("index.txt", std::ios_base::in);
    int idx;
    indexFileIn >> idx;
    indexFileIn.close();

    std::fstream indexFileOut("index.txt", std::ios_base::out);
    indexFileOut << (idx + 1);
    indexFileOut.close();

    // Prefix for experiment
    string fname_prefix = "f100RobotWCandCOpti";

    // Open file for logging rewards
    std::stringstream reward_fname;
    reward_fname << fname_prefix << "_r_" << idx << ".txt";

    yfile.open (reward_fname.str());

    // Open file for logging if it took optimal value
    std::stringstream isopti_fname;
    isopti_fname << fname_prefix << "_o_" << idx << ".txt";

    isoptimalfile.open(isopti_fname.str());

    // Intialize weights matrix
    for(int i = 0; i < 10; i++){
        for(int j = 0; j < 3; j++){
            estiQ[i][j] = 1;
        }
    }

    // Acces terminal for debug logging
    term = std::ofstream("/dev/tty", std::ios_base::out);

}

/****************************************/
/****************************************/

void CFootBotFeatureSelection::ControlStep() {

    messageHandling();
    clearMessage();

    switch (state) {
        case EXPLORE:{
            term << "Explore" << std::endl;
            explore();
            break;
        }
        case DRIVETOBOX:{
            term << "Drive to box" << std::endl;
            driveToBox();
            break;
        }
        case EXPERIMENT:{
            term << "Experiment" << std::endl;
            experiment();
            break;
        }
        case BACKUP:{
            term << "Backup" << std::endl;
            backup();
            break;
        }
        case STUCK:{
            term << "Stuck" << std::endl;
            stuck();
            break;
        }
        case FINISHED:{

            break;
        }
        default:{
            std::cout<<"error State: " << state <<std::endl;
            state = EXPLORE;
        }
    }

}
/****************************************/
/*        state action functions        */
/****************************************/
void CFootBotFeatureSelection::explore() {

    // Check if the camera detects any LEDs
    if(blobInSight()){
        state = DRIVETOBOX; // If LEDs are detected change state to Drive to Box(Object)
        return;
    };

    CRadians resetAngle;
    resetAngle.SetValue(0);
    cAngle = resetAngle;
    cAccumulator.FromPolarCoordinates(0,resetAngle);
    //std::cout<<"reset: Angle: "<<cAccumulator.Angle()<<" length "<<cAccumulator.Length()<<std::endl;

    /* Get readings from proximity sensor */
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

    /* Sum them together */
    for(size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }


    cAccumulator /= tProxReads.size();
//    std::cout<<"x: "<<cAccumulator.GetY()<<std::endl;
//    std::cout<<"y: "<<cAccumulator.GetY()<<std::endl;
//    std::cout<<"length: "<<cAccumulator.Length()<<std::endl;
//    std::cout<<"angle: "<<cAccumulator.Angle()<<std::endl;
    /* If the angle of the vector is small enough and the closest obstacle
     * is far enough, continue going straight, otherwise curve a little
     */
    CRadians cAngle = cAccumulator.Angle();

    //std::cout<<"Angle: "<<cAccumulator.Angle()<<" - Length: "<<cAccumulator.Length()<<std::endl;

    if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) && cAccumulator.Length() < m_fDelta ) {
        /* Go straight */
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
        //std::cout<<"go straight"<<std::endl;
    }
    else {
        /* Turn, depending on the sign of the angle */
        if(cAngle.GetValue() > 0.0f) {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
           // std::cout<<"turn right"<<std::endl;
        }
        else {
            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
           // std::cout<<"turn left"<<std::endl;
        }
    }
}

void CFootBotFeatureSelection::driveToBox() {

    //get sensor readings
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings &sBlobs = m_pcCamera->GetReadings();
    const CCI_DifferentialSteeringSensor::SReading &sDiffS = m_pcDS->GetReading();
    const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();

    // are the cylinder still in sight
    if (sBlobs.BlobList.empty()) {
        state = EXPLORE; // if not change to explore state
        return;
    }

    // convert readings to index
    for (auto &blob : sBlobs.BlobList) {
        blobColorIndex.push_back(convertColorToIndex(blob->Color));
    }



    //check if new cylinders has been detected
    for (auto &index:blobColorIndex) {
        auto pos = std::find(prevBlobColorIndex.begin(), prevBlobColorIndex.end(), index);
        if (pos == prevBlobColorIndex.end()) {
            chosenCylinder = chooseCylinderToDriveTo();
            break;
        }
    }

    // is the previously chosen still in sight
    auto pos = std::find(blobColorIndex.begin(), blobColorIndex.end(), chosenCylinder);
    if (pos == blobColorIndex.end()) {
        chosenCylinder = chooseCylinderToDriveTo();
    }

    if (chosenCylinder == -1) {
        state = EXPLORE;
        return;
    }

    //Find chosen cylinder in sensor readings
    int id = -1;
    for (int i = 0; i < blobColorIndex.size(); ++i) {
        if (blobColorIndex[i] == chosenCylinder) {
            id = i;
        }
    }
    //term << "dtb id : " << id << std::endl;
    //term << blobColorIndex.size() << " | " << sBlobs.BlobList.size() << std::endl;
    prevBlobColorIndex.swap(blobColorIndex);
    blobColorIndex.clear();

    const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob *chosenBlob = sBlobs.BlobList[id];
    sendMessage();

    // Stuck check
    if (abs(chosenBlob->Distance - prevBlobDist) < 0.1) {
        stuckCounter++;

    } else {
        stuckCounter = std::max(0, stuckCounter - 1);
    }

    if (stuckCounter >= 30) {
        state = STUCK;
        stuckCounter = 0;
        return;
    }
    //std::cout << "blob dist:prev "<< chosenBlob->Distance<<";"<< prevBlobDist << std::endl;
    //std::cout << "Stuck count: "<< stuckCounter << std::endl;
    prevBlobDist = chosenBlob->Distance;
    // End stuck check

    if (chosenBlob->Distance >= 20) {
        //if the robot is not close enough to the box avoid the obstacle;
        CRadians angleToNearest = chosenBlob->Angle;
        if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(angleToNearest) == 1) {
            m_pcWheels->SetLinearVelocity(0.5 * m_fWheelVelocity, 0.5 * m_fWheelVelocity);
            explore();
        } else {
            /* Turn, depending on the sign of the angle */
            if (angleToNearest.GetValue() < 0.0f) {
                m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity * 0.2);
            } else {
                m_pcWheels->SetLinearVelocity(m_fWheelVelocity * 0.2, m_fWheelVelocity);
            }
        }
    } else {
        state = EXPERIMENT;
    }
}

void CFootBotFeatureSelection::experiment() {

    //get sensor readings
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs =m_pcCamera->GetReadings();
    // Check the chosen cylinder is available
    int id = -1;
    for (int i = 0; i <sBlobs.BlobList.size() ; ++i) {
        int cylid = convertColorToIndex(sBlobs.BlobList[i]->Color);
        if(cylid == chosenCylinder){
            id = i;
        }
    }
    // If not or to far away change to Explore state
    if(id == -1 || sBlobs.BlobList[id]->Distance > 20){
        state = EXPLORE;
        return;
    }
    // The chosen cylinder
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob* chosenBlob = sBlobs.BlobList[id];

    double angle2cyl = chosenBlob->Angle.GetValue();
    double dist2cyl =  chosenBlob->Distance;

    // If not in suitable position to grasp the cylinder, move there
    if(chosenBlob->Distance > 18.54 || abs(angle2cyl) > 0.08) {
        double vel = 0.5 * m_fWheelVelocity * (dist2cyl - 18.2);
        double angleW = 0.3 * (1 - (abs(angle2cyl) / 3.15));
        if (angle2cyl < 0.0f) {
            m_pcWheels->SetLinearVelocity( vel, vel * angleW);
        } else {
            m_pcWheels->SetLinearVelocity(vel * angleW, vel);
        }

        // Stuck check
        if (abs(chosenBlob->Distance - prevBlobDist) < 0.1) {
            stuckCounter++;

        }
        else {
            stuckCounter = std::max(0, stuckCounter - 1);
        }

        if (stuckCounter >= 30){
            state = STUCK;
            stuckCounter = 0;
            return;
        }
        //std::cout << "blob dist:prev "<< chosenBlob->Distance<<";"<< prevBlobDist << std::endl;
        //std::cout << "Stuck count: "<< stuckCounter << std::endl;
        prevBlobDist = chosenBlob->Distance;
        // End stuck check

    }
    // When aligned, experiment and learn
    else{
        /* stop */
        m_pcWheels->SetLinearVelocity(0, 0);

        //Choosing action based on cylinder features.
        int currentState = chosenCylinder;
        int exploration = mgenerate(1,100);
        int action = -1;
        if(exploration > m_Epsilon){                                        //epsilon is set in the argos file. % exploration
            action = greedyAction(currentState);
        }else{
            // Explore choose random non optimal action
            action = exploreAction(currentState);

            // For test with 100% random action
            //action = mgenerate(0,9);
        }
        // Log if it took the true optimal value or not
        isoptimalfile << isOptimalAction(chosenCylinder, action) << " ";

        if(action != -1) {
            int rewards = reward(currentState, action);
            addData(lookUpPointer[chosenCylinder],rewards,action);
            sendMessage(lookUpPointer[chosenCylinder],rewards,action);
            //std::cout << "(s,a): (" << currentState << " , " << action << ")" << std::endl;
            //std::cout << "Reward: "<< rewards << std::endl;

            yfile <<rewards<<" ";
            nrOfExp++;
            LOG << "Nr. Exp.: " << nrOfExp << std::endl;

            if(nrOfExp>=500){
                m_pcWheels->SetLinearVelocity(0, 0);
                printQtable();
                state = FINISHED;
                std::cout << "Are you finnish - Yes i am finnish" << std::endl;
                return;
            }

            collectedReward += rewards;
            if(experience[action].size() >= 3)
               updateQ(action);


            if(rewards == 1) {
                m_pcGripper->LockPositive();

                //std::cout << "Collected Reward: " << collectedReward << std::endl

                state = BACKUP;
            }
        } else{
            state = EXPLORE;
        }

    }

}


void CFootBotFeatureSelection::backup() {
    m_pcGripper->Unlock();
    //sendMessage();
    printQtable();
    //m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, -m_fWheelVelocity);
    state = EXPLORE;
}

//Check if a cylinder is in sight//
bool CFootBotFeatureSelection::blobInSight() {
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs =m_pcCamera->GetReadings();

    for( auto blob:sBlobs.BlobList){
        int cylID = convertColorToIndex(blob->Color);
        auto pos = std::find(takenCylinders.begin(),takenCylinders.end(),cylID);
        if(pos == takenCylinders.end()){
            return true;
        }
    }
    return false;
}

/****************************************/
/*     Functions for learning      */
/****************************************/

int CFootBotFeatureSelection::convertColorToIndex(CColor cylColor) {
    int fIndex = cylColor.GetGreen()<<8 | cylColor.GetBlue();
    return fIndex;
}

void CFootBotFeatureSelection::addData(const double *featureVector,int reward, int action) {
    std::array<double,4> data = {featureVector[0], featureVector[1],featureVector[2], (double)reward};
    experience[action].push_back(data);
}

int CFootBotFeatureSelection::chooseCylinderToDriveTo() {

    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& Blobs =m_pcCamera->GetReadings();

    std::vector<int> availCyl;
    for(auto blob:Blobs.BlobList){
        int  cylID = convertColorToIndex(blob->Color);
        auto pos = std::find(takenCylinders.begin(),takenCylinders.end(),cylID);
        if (pos == takenCylinders.end()){
            availCyl.push_back(cylID);
        }
    }

    if (availCyl.empty()){
        return -1;
    }

    if(m_Epsilon==100){
       int id = mgenerate(0, availCyl.size()-1);
       int randomCyl = availCyl[id];
       return randomCyl;
    }

    int bestCylinderIndex = -1;
    double bestCylEsti = -1;

    for (int featureIndex : availCyl) {
        double cylEsti = cylinderBestEsti(lookUpPointer[featureIndex]);
        if (cylEsti>bestCylEsti){
            bestCylEsti=cylEsti;
            bestCylinderIndex = featureIndex;
        }
    }
    return bestCylinderIndex;
}

double CFootBotFeatureSelection::cylinderBestEsti(const double *featureVector) {
    double bestChance = -1;
    for(int i = 0; i<10; i++){
        double cylEsti= 0;
        for(int j = 0; j<featureLength; j++){
            double feat = featureVector[j];
            cylEsti += estiQ[i][j]*feat;
        }
        if(cylEsti>bestChance){
            bestChance = cylEsti;
        }
    }
    return bestChance;
}


int CFootBotFeatureSelection::reward(int cIndex, int action) {

    double chanceOfSuccess = 0;
    for (int i = 0; i<featureLength; i++){
        chanceOfSuccess += lookUpPointer[cIndex][i] * weightTable[action][i];
    }

    int randNr = mgenerate(1,100);
    chanceOfSuccess *= 100;
    //term << "c index "<<cIndex<<"; Action "<<action<<"; success "<< chanceOfSuccess<< std::endl;

    if(randNr <= chanceOfSuccess){
        return 1;
    } else{
        return 0;
    }
}


void CFootBotFeatureSelection::updateQ(int action) {
    unsigned int n = experience[action].size();
    MatDoub A(n,featureLength);
    VecDoub b(n);
    VecDoub x(featureLength);
    for (int r = 0; r < n ; r++){
        for (int c = 0; c < featureLength; ++c) {
            A[r][c] = experience[action][r][c];
        }
        b[r] = experience[action][r][featureLength];
    }
    SVD svd(A);
    svd.solve(b,x);

    for(int f = 0; f < x.size(); f++){
        estiQ[action][f] = x[f];
    }

}

int CFootBotFeatureSelection::greedyAction(int cylinder) {

    std::vector<int> actions;
    actions.clear();
    double highestQ = 0;

    for(int i = 0; i < 10 ; i++){
        double cylEsti = 0;
        for (int j=0; j<featureLength; j++){
            cylEsti += estiQ[i][j]*lookUpPointer[cylinder][j];
        }
        if (cylEsti==highestQ){
            actions.push_back(i);
        }
        if (cylEsti>highestQ){
            highestQ=cylEsti;
            actions.clear();
            actions.push_back(i);
        }

    }

    int randNr = mgenerate(0,actions.size()-1);

    if (!actions.empty()){
        return actions[randNr];
    }else{
        std::cout<<"no action chosen"<<std::endl;
    }
    return -1;
}



void CFootBotFeatureSelection::printQtable() {
    std::cout<<"Q-Table: {"<<std::endl;
    std::cout<<std::setprecision(2)<<std::endl;
    for (int i = 0; i < 10; ++i) {
        std::cout<<"{ ";
        for(int j = 0; j < featureLength; ++j){
            if (j != 0) std::cout << ", ";
            std::cout << std::fixed << estiQ[i][j];
        }
        std::cout<<"} " << "exp: " << experience[i].size() <<std::endl;
    }
}


int CFootBotFeatureSelection::exploreAction(int state) {
    int ga = greedyAction(state);

    std::vector<int> posAct;
    posAct.clear();
    for (int i=0;i<10;i++){
        if(i!=ga){
            posAct.push_back(i);
        }
    }
    int action = posAct[mgenerate(0,posAct.size())];
    return action;
}

/****************************************/
/*     Functions for communication      */
/****************************************/
void CFootBotFeatureSelection::sendMessage( const double *featureVector,int reward, int action) {

    m_pcRABA->SetData(0,1);
    m_pcRABA->SetData(1, chosenCylinder);

    for (int i = 0; i < featureLength; ++i) {
        m_pcRABA->SetData(i + 2, char(featureVector[i] * 100.0));
    }
    m_pcRABA->SetData(featureLength + 2, reward);
    m_pcRABA->SetData(featureLength + 3, action);

}

void CFootBotFeatureSelection::sendMessage() {
    m_pcRABA->SetData(0,2);
    m_pcRABA->SetData(1, chosenCylinder);
}

void CFootBotFeatureSelection::messageHandling() {

    const CCI_RangeAndBearingSensor::TReadings& tMessages =m_pcRABS->GetReadings();
    double* featureVector = new double[featureLength];
    takenCylinders.clear();
    for (const auto & tMessage : tMessages) {
        if (tMessage.Data[0]>0){
            takenCylinders.push_back(tMessage.Data[1]);
        }

        if (tMessage.Data[0] == 1) {
            for (int i = 0; i < 7; ++i) {
                cout << tMessage.Data[i] << ", ";
            }
            cout << std::endl;

            for (int i = 0; i < featureLength; ++i) {
                featureVector[i] = (double)(tMessage.Data[i + 2]) / 100.0;
            }
            int action = tMessage.Data[featureLength + 3];
            addData(featureVector, tMessage.Data[featureLength + 2] , action);
            if(experience[action].size() >= 3) {
                updateQ(action);
            }
        }
    }
    delete[] featureVector;
}

void CFootBotFeatureSelection::clearMessage() {
    m_pcRABA->ClearData();
}

void CFootBotFeatureSelection::stuck() {
    if (stuckRuns == 0) {
        m_pcWheels->SetLinearVelocity(-4, 0);
    }
    if (stuckRuns == 37) {
        m_pcWheels->SetLinearVelocity(-4, -4);
    }

    if (stuckRuns == 60) {
        m_pcWheels->SetLinearVelocity(-0, -6);
    }

    stuckRuns++;
    if(stuckRuns > 75){
        state = EXPLORE;
        stuckRuns = 0;
    }

}

void CFootBotFeatureSelection::Destroy() {
    if(nrOfExp == 0){
        std::fstream indexFileIn("index.txt", std::ios_base::in);
        int idx;
        indexFileIn >> idx;
        indexFileIn.close();

        std::fstream indexFileOut("index.txt", std::ios_base::out);
        indexFileOut << (idx - 1);
        indexFileOut.close();
    }

    yfile.close();
    CCI_Controller::Destroy();
}

void CFootBotFeatureSelection::setLookUpPointer(const double * const * lUp, unsigned int fLength) {
    lookUpPointer = lUp;
    featureLength = fLength;
}

int CFootBotFeatureSelection::mgenerate(int first, int last) {

    std::uniform_real_distribution<double> distribution(first, last);
    std::random_device rd;
    std::default_random_engine generator(rd());
    return distribution(generator);
}

bool CFootBotFeatureSelection::isOptimalAction(int cIndex, int action) {
    std::vector<int> optimal_actions;
    double best_chance = 0;
    for (int a = 0; a < 10; ++a) {
        double chanceOfSuccess = 0;
        for (int f = 0; f < featureLength; f++) {
            chanceOfSuccess += lookUpPointer[cIndex][f] * weightTable[a][f];
        }
        if (chanceOfSuccess == best_chance){
            optimal_actions.push_back(a);
        }
        if (chanceOfSuccess > best_chance){
            best_chance = chanceOfSuccess;
            optimal_actions.clear();
            optimal_actions.push_back(a);
        }
    }
    for (auto& optimal_action : optimal_actions) {
        if(optimal_action == action) return true;
    }
    return false;
}




/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotFeatureSelection, "footbot_featureselection_controller")
