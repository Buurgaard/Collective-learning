numfiles = 200;
mydata = cell(1,numfiles);
x = linspace(1,500,500);
movavg = 75;

%%----------------------100 greedy----------------------------------------------------

for k = 1:numfiles
    myfilename = sprintf('argos3-examples/matlab/data/EC/greedy/ECGreedyfb%d.txt',k-1);
    mydataEC100GR{k} = importdata(myfilename);
end

averageRewardEC100GR = zeros(1,500);

for i = 1:numfiles
    averageRewardEC100GR = averageRewardEC100GR + mydataEC100GR{i};
end

averageRewardEC100GR = averageRewardEC100GR/numfiles;

movavgEC100GR=movmean(averageRewardEC100GR,movavg);

%%---------------------------optimictic initital value 1----------------------------------------------
for k = 1:numfiles
    myfilename = sprintf('argos3-examples/matlab/data/EC/opti/ECOptifb%d.txt',k-1);
    mydataECOpIn{k} = importdata(myfilename);
end

averageRewardECOpIn = zeros(1,500);

for i = 1:numfiles
    averageRewardECOpIn = averageRewardECOpIn + mydataECOpIn{i};
end
averageRewardECOpIn = averageRewardECOpIn/numfiles;

movavgECOpIn=movmean(averageRewardECOpIn,movavg);

%%---------------------epsilon 10-----------------------------------------------------
for k = 1:numfiles
    myfilename = sprintf('argos3-examples/matlab/data/EC/10/ECGreedy10fb%d.txt',k-1);
    mydataEC10GR{k} = importdata(myfilename);
end

averageRewardEC10GR = zeros(1,500);

for i = 1:numfiles
    averageRewardEC10GR = averageRewardEC10GR + mydataEC10GR{i};
end
averageRewardEC10GR = averageRewardEC10GR/numfiles;

movavgEC10GR=movmean(averageRewardEC10GR,movavg);


%%--------------------------Random------------------------------------------------

for k = 1:numfiles
    myfilename = sprintf('argos3-examples/matlab/data/EC/random/ECrandomfb%d.txt',k-1);
    mydataECR{k} = importdata(myfilename);
end

averageRewardECR = zeros(1,500);

for i = 1:numfiles
    averageRewardECR = averageRewardECR + mydataECR{i};
end
averageRewardECR = averageRewardECR/numfiles;

movavgECR=movmean(averageRewardECR,movavg);

%%-------------------------------------------------------------------------

plot(x,movavgEC100GR,'r',x,movavgECOpIn,'b',x,movavgEC10GR,'g',x,movavgECR,'y');
ylim([0 1.0]);