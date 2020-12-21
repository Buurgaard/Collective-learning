/* Include the controller definition */
#include "footbot_areacleaning.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>




/****************************************/
/****************************************/

CFootBotAreaCleaning::CFootBotAreaCleaning() :
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

void CFootBotAreaCleaning::Init(TConfigurationNode& t_node) {
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

/*
    std::string idnr = GetId();
    std::string id="";
    for(int i = 2; i<idnr.length();i++){
        if(idnr.length()==3){
            id.push_back('0');
        }
        id.push_back(idnr[i]);
    }*/
    // Index magic
    std::fstream indexFileIn("index.txt", std::ios_base::in);
    int idx;
    indexFileIn >> idx;
    indexFileIn.close();

    std::fstream indexFileOut("index.txt", std::ios_base::out);
    indexFileOut << (idx + 1);
    indexFileOut.close();

    std::stringstream filename;

    filename << "100Comfb" << idx << ".txt";

    yfile.open (filename.str());

    float reward2Table[3][10] = {{90, 50, 65, 40,  5, 30, 20, 10,  5, 2},
                              { 25, 65,  1, 50, 75, 15,  1, 30, 20, 4},
                              {  1, 10, 30, 30, 20, 10,  5, 15, 25, 4}}; //rows, cols

    for(int i = 0; i < 3; i++){
        for(int j = 0; j<10; j++){
            estiQ[i][j]=1;
            nrSAPairChosen[i][j] = 1;
        }
    }


    printQtable();

}

/****************************************/
/****************************************/

void CFootBotAreaCleaning::ControlStep() {

    messageHandling();
    clearMessage();
    switch (state) {
        case EXPLORE:{
            //std::cout << "Explore" << std::endl;
            explore();
            break;
        }
        case DRIVETOBOX:{
            //std::cout << "Drive to box" << std::endl;
            driveToBox();
            break;
        }
        case EXPERIMENT:{
            //std::cout << "Experiment" << std::endl;
            experiment();
            break;
        }
        case BACKUP:{
            //std::cout << "Backup" << std::endl;
            backup();
            break;
        }
        case STUCK:{
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
void CFootBotAreaCleaning::explore() {


    if(blobInSight()==true){
        prevBlobDist = 0;
        state = DRIVETOBOX; //drive to box
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

void CFootBotAreaCleaning::driveToBox() {

    //find the nearest box by color//
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs =m_pcCamera->GetReadings();
    const CCI_DifferentialSteeringSensor::SReading& sDiffS = m_pcDS->GetReading();
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

    /*if(tProxReads[0].Value == 1 && tProxReads[23].Value == 1){
        if (sDiffS.VelocityLeftWheel-sDiffS.VelocityRightWheel == 0){
            state = STUCK;
            return;
        }
    }*/


    if(sBlobs.BlobList.empty()){
        state = EXPLORE; //explore state
        return;
    }

    if (newColorsInSight(colorInSight(CColor::RED), colorInSight(CColor::GREEN),
                         colorInSight(CColor::BLUE))) {
        //std::cout<<"new color"<<std::endl;
        chosenColor = chooseColorToDriveTo(colorInSight(CColor::RED), colorInSight(CColor::GREEN),
                                           colorInSight(CColor::BLUE));
    }

    float nearestBlobDis;

    int nearestBlobIdx = nearestBlob(chosenColor, nearestBlobDis);

    if (abs(nearestBlobDis - prevBlobDist) < 0.04)
        stuckCounter++;
    else
        stuckCounter = std::min(0, stuckCounter - 1);

    if (stuckCounter >= 8){
        state = STUCK;
        stuckCounter = 0;
        return;
    }

    prevBlobDist = nearestBlobDis;


    if(nearestBlobIdx == -1){
        std::cerr << "No matching box found!" << std::endl;
        colorMem[0] = false;
        colorMem[1] = false;
        colorMem[2] = false;
        state = EXPLORE; // explore();
        return;
    }

    //mess << sBlobs.BlobList[nearestBlobIdx]->Color;
    //steer after the nearest box
    //std::cout << nearestBlobDis << std::endl;

    if(nearestBlobDis > 20){
        //if the robot is not close enough to the box avoid the obstacle;
        CRadians angleToNearest = sBlobs.BlobList[nearestBlobIdx]->Angle;
        if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(angleToNearest) == 1 ) {
            //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
            explore();
        }
        else{
            /* Turn, depending on the sign of the angle */
            if(angleToNearest.GetValue() < 0.0f) {
                m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
            }
            else {
                m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
            }
        }
    }
    else if(nearestBlobDis < 19) {
        state = EXPERIMENT;
    }else{
        /* Turn, depending on the sign of the angle */
        CRadians angleToNearest = sBlobs.BlobList[nearestBlobIdx]->Angle;
        if(angleToNearest.GetValue() < 0.0f) {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
        }
        else {

            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
        }

    }

}

void CFootBotAreaCleaning::experiment() {
    float dist;
    if(nearestBlob(chosenColor, dist) == -1 || dist >= 19){
        state = EXPLORE;
        return;
    }



    /* Get readings from proximity sensor */
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    /* Sum them together */
    CVector2 cAccumulator;

    for(size_t i = 0; i < tProxReads.size(); ++i) {
        cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }

    cAccumulator /= tProxReads.size();
    /* If the angle of the vector is small enough and the closest obstacle
     * is far enough, continue going straight, otherwise curve a little
     */
    CRadians cAngle = cAccumulator.Angle();

    //std::cout<<"length: "<<cAccumulator.Length()<<std::endl;
    //std::cout<<"angle: "<<cAccumulator.Angle()<<std::endl;
    /* Get readings from proximity sensor */
    float centerDelta = tProxReads[0].Value-tProxReads[23].Value;
    float center = 0.1;
    if(centerDelta > center){
        m_pcWheels->SetLinearVelocity(-0.5, 0.5f);
        //std::cout<<"right "<<centerDelta<<std::endl;
    }
    if(centerDelta < -center){
        m_pcWheels->SetLinearVelocity(0.5f, -0.5);
        //std::cout<<"left "<<centerDelta<<std::endl;
    }

    if(abs(centerDelta) <= center && tProxReads[0].Value > 0 ){
        /* stop */
        m_pcWheels->SetLinearVelocity(0, 0);

        int currentState = convertColorToState(chosenColor);
        int exploration = mgenerate(1,100);
        int action = -1;
        if(exploration > m_Epsilon){
            action = greedyAction(currentState);
        }else{
            action = exploreAction(currentState);
            //action = mgenerate(0,9);
        }

        if(action != -1) {
            int rewards = reward(currentState, action);
            //std::cout << "(s,a): (" << currentState << " , " << action << ")" << std::endl;
            //std::cout << "Reward: "<< rewards << std::endl;

            yfile <<rewards<<" ";
            nrOfExp++;
            if(nrOfExp>=500){
                m_pcWheels->SetLinearVelocity(0, 0);
                printQtable();
                state = FINISHED;
                std::cout << "Are you finnish - Yes i am finnish" << std::endl;
                return;
            }

            collectedReward += rewards;
            updateQ(currentState,action,rewards);

            tempnrSAPairChosen[action]++;



            if(rewards == 1) {
                m_pcGripper->LockPositive();
                colorMem.at(0) = false;
                colorMem.at(1) = false;
                colorMem.at(2) = false;
                //std::cout << "Collected Reward: " << collectedReward << std::endl;

                sendMessage(tempnrSAPairChosen, currentState, action);
                for (int i = 0; i < 10; ++i) {
                    tempnrSAPairChosen[i] = 0;
                }

                state = BACKUP;
            }
        } else{
            state = EXPLORE;
        }

    }

}


void CFootBotAreaCleaning::backup() {
    m_pcGripper->Unlock();
    //sendMessage();
    printQtable();
    //m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, -m_fWheelVelocity);
    state = EXPLORE;
}

//Check if a cylinder is in sight//
bool CFootBotAreaCleaning::blobInSight() {
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs =m_pcCamera->GetReadings();
    if (sBlobs.BlobList.size() > 0){
        return true;
    }
    return false;
}

// Function to check if a certain color is in sight//
bool CFootBotAreaCleaning::colorInSight(CColor color) {
    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& Blobs =m_pcCamera->GetReadings();
    for(size_t i = 0; i < Blobs.BlobList.size(); ++i) {
        CColor blobColor = Blobs.BlobList[i]->Color;
        if (blobColor == color) {
            return true;
        }
    }
    return false;
}


/****************************************/
/*     Functions for learning      */
/****************************************/

CColor CFootBotAreaCleaning::chooseColorToDriveTo(bool r, bool g, bool b) {

    std::vector<CColor> colors;
    std::vector<float> bestColor;
    colors.empty();
    bestColor.empty();


    float qRed = 0;
    float qBlue = 0;
    float qGreen = 0;

    if(r==true){
        colors.push_back(CColor::RED);
        for(int i = 0; i<10;i++){
            float qr = estiQ[0][i];
            if(qr>qRed){
                qRed=qr;
            }
        }
        bestColor.push_back(qRed);
    }
    if(b==true){
        colors.push_back(CColor::BLUE);
        for(int i = 0; i<10;i++) {
            float qb = estiQ[1][i];
            if (qb > qBlue) {
                qBlue = qb;
            }
        }
        bestColor.push_back(qBlue);
    }
    if(g==true){
        colors.push_back(CColor::GREEN);
        for(int i = 0; i<10;i++){
            float qg = estiQ[2][i];
            if(qg>qGreen){
                qGreen=qg;
            }
        }
        bestColor.push_back(qGreen);
    }

    if(colors.size()==1){
        return colors[0];
    }

    int randomNr = mgenerate(1,100);
    if (randomNr<0){
        return colors[mgenerate(0,colors.size()-1)];
    }

    int bestIndex = 0;
    float index = 0;
    for (int j = 0; j < colors.size(); ++j) {
        float colorv = bestColor[j];
        if(colorv>index){
            index = colorv;
            bestIndex = j;
        }
    }

    return colors[bestIndex];

//
//    //std::cout<<"rand "<<randomNr<<" , "<<colors.size()<<std::endl;
//    if(!colors.empty()) {
//        return colors[randomNr];
//    }

}

bool CFootBotAreaCleaning::newColorsInSight(bool r, bool g, bool b) {

    bool change = false;

    if(r != colorMem.at(0)) {
        colorMem.at(0) = r;
        if (r == true) {
            change = true;
        }
    }

    if(g != colorMem.at(1)){
        colorMem.at(1) = g;
        if(g == true){
            change = true;
        }
    }
    if(b != colorMem.at(2)){
        colorMem.at(2) = b;
        if(b == true){
            change = true;
        }
    }

        return change;
}


int CFootBotAreaCleaning::reward(int state, int action) {
    //thee state in front of red, blue or green. Those action soft, medium, or hard grab//
    //contains number between 1 and 100 deffining chance of success.
    int rewardTable[3][10] = {{90, 50, 65, 40,  5, 30, 20, 10,  5, 2},
                             { 25, 65,  1, 50, 75, 15,  1, 30, 20, 4},
                             {  1, 10, 30, 30, 20, 10,  5, 15, 25, 4}}; //rows, cols

    int chanceOfSuccess = rewardTable[state][action];

//    int randNr;
//    for (int index = 0; index < 5; index++) {
//        randNr = (rand() % 100) + 1;
//    }


    int randNr = mgenerate(1,100);

    //std::cout<<"random: "<< randNr<<std::endl;
    //std::cout<<"Chance of success: "<< chanceOfSuccess<<std::endl;
    if(randNr <= chanceOfSuccess){
        return 1;
    } else{
        return 0;
    }
}

void CFootBotAreaCleaning::saUpdateNrOfTaken(int state, int action, int nrTimes = 1) {
    nrSAPairChosen[state][action] += nrTimes;
}

int CFootBotAreaCleaning::numberOfTimesTaken(int state, int action) {
    return nrSAPairChosen[state][action];
}

void CFootBotAreaCleaning::updateQ(int state, int action, int reward) {
    // NEW_estimate = OLD_estimate + stepsize(reward - OLD_estimate);
    saUpdateNrOfTaken(state,action);
    float n = numberOfTimesTaken(state,action);
    float r = reward;

    if(nrSAPairChosen[state][action] != 0 ){

        estiQ[state][action] = estiQ[state][action] + (1/n)*(r - estiQ[state][action]);
    }


}

int CFootBotAreaCleaning::greedyAction(int state) {
    //1-RED , 2-BLUE , 3-GREEN
    std::vector<int> actions;
    actions.clear();
    float highestQ = 0;

    for(int i = 0; i < 10 ; i++){
        float q = estiQ[state][i];
        if( q == highestQ){
            actions.push_back(i);

        }if (q > highestQ){
            actions.clear();
            actions.push_back(i);
            highestQ = q;
        }


    }
    //std::cout<<"actions vector";
    for(int j = 0 ; j<actions.size();j++){
        //std::cout<<actions[j]<<", ";
    }
    //std::cout<<std::endl;

    int randNr = mgenerate(0,actions.size()-1);
    //std::cout<<"random: "<< randNr<<std::endl;
    //std::cout<<"size: "<< actions.size()<<std::endl;
    if (actions.size() != 0){
        return actions[randNr];
    }else{
        std::cout<<"no action chosen"<<std::endl;
    }
    return -1;
}

int CFootBotAreaCleaning::convertColorToState(CColor cstate) {
    //1-RED , 2-BLUE , 3-GREEN
    if(cstate == CColor::RED){
        return 0;
    }
    if(cstate == CColor::BLUE){
        return 1;
    }
    if(cstate == CColor::GREEN){
        return 2;
    }
    return -1;
}

void CFootBotAreaCleaning::printQtable() {
    std::cout<<"Q-Table: {"<<std::endl;
    std::cout<<std::setprecision(2)<<std::endl;
    for (int i = 0; i < 3; ++i) {
        std::cout<<"{ ";
        for(int j = 0; j < 10; ++j){
            std::cout << std::fixed << estiQ[i][j] <<", ";
        }
        std::cout<<"}"<<std::endl;
    }
}

int CFootBotAreaCleaning::mgenerate(int first, int last) {

    std::uniform_real_distribution<double> distribution(first, last);
    std::random_device rd;
    std::default_random_engine generator(rd());
    return distribution(generator);
}

int CFootBotAreaCleaning::nearestBlob(CColor color, float &dist) {
    int nearestBlob = -1;
    dist = 10000.f;

    const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobs =m_pcCamera->GetReadings();

    for (size_t i = 0; i < sBlobs.BlobList.size(); ++i) {

        float tdist = sBlobs.BlobList[i]->Distance;
        CColor tcolor = sBlobs.BlobList[i]->Color;
        //std::cout << "chosen: "<<chosenColor << std::endl;

        if (tdist < dist && tcolor == color) {
            dist = tdist;
            nearestBlob = i;
            //std::cout << "going for: "<<chosenColor << std::endl;
        }
    }
    //std::cout << "nearst blob dist: "<< nearestBlobDis << std::endl;

    return nearestBlob;
}

void CFootBotAreaCleaning::Destroy() {
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

int CFootBotAreaCleaning::exploreAction(int state) {
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
void CFootBotAreaCleaning::sendMessage(float* nums, int& s, int& action) {
    m_pcRABA->SetData(0, (s << 6) | action);

    for (int i = 0; i < 10; ++i) {
        m_pcRABA->SetData(i+1,nums[i]);
    }
}
void CFootBotAreaCleaning::sendMessage() {


    /*
    int dataidx = 0;
    for (int i = 0; i<3 ; i++ ){
        float bestQ = 0;
        int   index = 0;
        for (int j = 0; j < 10; ++j) {
            float q = estiQ[i][j];
            if (q>bestQ){
                bestQ = q;
                index = j;
            }
        }
        int nr = nrSAPairChosen[i][index];
        if (nr<255) {
            m_pcRABA->SetData(dataidx,index);
            dataidx++;
            int best = bestQ*100;
            m_pcRABA->SetData(dataidx,best);
            dataidx++;
            m_pcRABA->SetData(dataidx,nr);
            dataidx++;
            std::cout<<"idx: "<<index<<" Best Q: "<<bestQ<<" NR: "<<nr<<std::endl;

        } else{
            std::cout<<"Nr is too large"<<std::endl;
            m_pcRABA->SetData(dataidx, index);
            dataidx++;
            m_pcRABA->SetData(dataidx,bestQ);
            dataidx++;
            m_pcRABA->SetData(dataidx,255);
            dataidx++;
        }
    }
            m_pcRABA->SetData(9,0);
    */

}

void CFootBotAreaCleaning::messageHandling() {

    const CCI_RangeAndBearingSensor::TReadings& tMessages =m_pcRABS->GetReadings();
    for (const auto & tMessage : tMessages) {
        unsigned char s = tMessage.Data[0] >> 6;
        unsigned char a = tMessage.Data[0] & 0x3F;

        if(a >= 10 || s >= 3){
            std::cerr << "ERROR Parsing message" << (int)s << " | " << (int)a << std::endl;
        } else {

            for (int j = 0; j < 10; ++j) {
                float m = tMessage.Data[j + 1];
                if (m < 1) continue;
                float n = numberOfTimesTaken(s, a);
                estiQ[s][j] *= n / (n + m);
                if (j == a) estiQ[s][j] += 1 / (n + m);
                saUpdateNrOfTaken(s,j,tMessage.Data[j + 1]);
            }
        }
    }
//-------------------------------------------------------------
    /*for (int i = 0; i<tMessages.size(); i++) {
        int idxR        = tMessages[i].Data[0];
        float bestQR    = (float)(tMessages[i].Data[1])/100.0f;
        float messNrR   = (float) tMessages[i].Data[2];

        int idxB        = tMessages[i].Data[3];
        float bestQB    = (float)(tMessages[i].Data[4])/100.0f;
        float messNrB   = tMessages[i].Data[5];

        int idxG        = tMessages[i].Data[6];
        float bestQG    = (float)(tMessages[i].Data[7])/100.0f;
        float messNrG   = (float) tMessages[i].Data[8];


        if(messNrR!=0 ) {
            estiQ[0][idxR] = ((estiQ[0][idxR] * nrSAPairChosen[0][idxR]) + (bestQR * messNrR)) / (messNrR + nrSAPairChosen[0][idxR]);
            nrSAPairChosen[0][idxR] = nrSAPairChosen[0][idxR] + messNrR;

        }
        if(messNrB!=0 ) {
            estiQ[1][idxB] = ((estiQ[1][idxB] * nrSAPairChosen[1][idxB]) + (bestQB * messNrB)) / (messNrB + nrSAPairChosen[1][idxB]);
            nrSAPairChosen[1][idxB] = nrSAPairChosen[1][idxB] + messNrB;
            //std::cout<< idxB<<" "<<bestQB<<" "<<messNrB<<std::endl;
        }
        if(messNrG!=0 ) {
            estiQ[2][idxG] = ((estiQ[2][idxG] * nrSAPairChosen[2][idxG]) + (bestQG * messNrG)) / (messNrG + nrSAPairChosen[2][idxG]);
            nrSAPairChosen[2][idxG] = nrSAPairChosen[2][idxG] + messNrG;
        }
        for (int j = 0; j < 3; ++j) {
            if (isnanf(estiQ[j][idxG])){
                std::cout << "ERROR NaN occured: " << tMessages[i].Data[j*3] << tMessages[i].Data[j*3 + 1] << tMessages[i].Data[j*3 +2] << std::endl;
            }
        }

        //std::cout<<GetId()<<std::endl;
        //printQtable();

    }
    */
}

void CFootBotAreaCleaning::clearMessage() {
    m_pcRABA->ClearData();
}

void CFootBotAreaCleaning::stuck() {
    if (stuckRuns == 0) {
        std::cout << "stuck" << std::endl;
        m_pcWheels->SetLinearVelocity(-5, 0);
    }
    if (stuckRuns > 50){
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity,m_fWheelVelocity);
    }

    if(stuckRuns > 75){
        state = EXPLORE;
        stuckRuns = 0;
    }
    stuckRuns++;
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
REGISTER_CONTROLLER(CFootBotAreaCleaning, "footbot_areacleaning_controller")
