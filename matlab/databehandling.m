numfiles = 200;
mydata = cell(1,numfiles);
x = linspace(1,500,500);
movavg = 100;

%%----------------------100 greedy----------------------------------------------------
mydata100GR = cell(1,numfiles);
for k = 1:numfiles
    myfilename = sprintf('argos3-examples/matlab/data/100/RewardGreedyfb%d.txt',k-1);
    mydata100GR{k} = importdata(myfilename);
end

averageReward100GR = zeros(1,500);

for i = 1:numfiles
    averageReward100GR = averageReward100GR + mydata100GR{i};
end

averageReward100GR = averageReward100GR/numfiles;

movavg100GR=movmean(averageReward100GR,movavg);

%%---------------------------optimictic initital value 1----------------------------------------------
mydataOpIn = cell(1,numfiles);
for k = 1:numfiles
    myfilename = sprintf('argos3-examples/matlab/data/OpInValue/optimisticInitfb%d.txt',k-1);
    mydataOpIn{k} = importdata(myfilename);
end

averageRewardOpIn = zeros(1,500);

for i = 1:numfiles
    averageRewardOpIn = averageRewardOpIn + mydataOpIn{i};
end
averageRewardOpIn = averageRewardOpIn/numfiles;

movavgOpIn=movmean(averageRewardOpIn,movavg);

%%---------------------epsilon 10-----------------------------------------------------
mydata10GR = cell(1,numfiles);
for k = 1:numfiles
    myfilename = sprintf('argos3-examples/matlab/data/10/epsilon10greedyfb%d.txt',k-1);
    mydata10GR{k} = importdata(myfilename);
end

averageReward10GR = zeros(1,500);

for i = 1:numfiles
    averageReward10GR = averageReward10GR + mydata10GR{i};
end
averageReward10GR = averageReward10GR/numfiles;

movavg10GR=movmean(averageReward10GR,movavg);


%%--------------------------epsilon 20------------------------------------------------
mydataR = cell(1,numfiles);

for k = 1:numfiles
    myfilename = sprintf('argos3-examples/matlab/data/random/randomfb%d.txt',k-1);
    mydataR{k} = importdata(myfilename);
end

averageRewardR = zeros(1,500);

for i = 1:numfiles
    averageRewardR = averageRewardR + mydataR{i};
end
averageRewardR = averageRewardR/numfiles;

movavgR=movmean(averageRewardR,movavg);

%%-------------------------------------------------------------------------

plot(x,movavg100GR,'r',x,movavgOpIn,'b',x,movavg10GR,'g',x,movavgR,'y');
ylim([0 1.0]);