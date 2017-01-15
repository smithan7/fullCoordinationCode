close all
clear all
clc

% load file
filename = '/home/rdml/coordination_testing/fullCoordinationCode/Results/Visual_inf_27_dec_2016.csv';
delimiterIn = ',';
A = importdata(filename,delimiterIn);

% detect each individual exploration
int_cntr = 1;
begin = 1;
for i=2:length(A.data(:,1))
    if A.data(i,1) < A.data(i-1,1)
        segment{int_cntr}.data = A.data(begin:i-1,:);
        segment{int_cntr}.map = A.textdata(i-1,1);
        segment{int_cntr}.inference = A.textdata(i-1,2);
        segment{int_cntr}.coordination = A.textdata(i-1,3);
        int_cntr = int_cntr+1;
        begin = i;
    end
end

% normalize length
nTime = 500;
for i=1:length(segment(1,:))
    segment{i}.nData(:,1) = 0:nTime;
    for v=2:6
        segment{i}.nData(:,v) = interp1( segment{i}.data(:,1)/segment{i}.data(end,1), segment{i}.data(:,v), segment{i}.nData(:,1)/nTime );
    end
    segment{i}.nData(:,7) = (segment{i}.nData(:,5)-segment{i}.nData(:,6))./segment{i}.nData(:,6);
end

for t=1:nTime+1
    for v=2:7
        for s=1:length( segment )
            track(s) = segment{s}.nData(t,v);
        end
        means(t,v) = mean(track);
        vars(t,v) = std(track)/sqrt(length(segment));
    end
end

figure %recall
hold all
plot(segment{1}.nData(:,1)/nTime, means(:,5), 'r', 'linewidth',2)
plot(segment{1}.nData(:,1)/nTime, means(:,6), 'g', 'linewidth',2)

ylabel('Recall')
xlabel('Exploration Length')
legend('Inferred', 'Naive')

errorbar(segment{1}.nData(1:50:end,1)/nTime, means(1:50:end,5), vars(1:50:end,5), 'k.', 'linewidth',2)
errorbar(segment{1}.nData(1:50:end,1)/nTime, means(1:50:end,6), vars(1:50:end,6), 'k.', 'linewidth',2)
axis([0 1 0 1])

figure %gain in recall
hold all
plot(segment{1}.nData(:,1)/nTime, means(:,7), 'r', 'linewidth',2)

ylabel('Increase in Recall')
xlabel('Exploration Length')

errorbar(segment{1}.nData(1:50:end,1)/nTime, means(1:50:end,7), vars(1:50:end,7), 'k.', 'linewidth',2)
axis([0 1 0 1.1])

figure %precision
hold all
plot(segment{1}.nData(:,1)/nTime, means(:,2), 'r', 'linewidth',2)
plot(segment{1}.nData(:,1)/nTime, means(:,4), 'g', 'linewidth',2)

ylabel('Precision')
xlabel('Exploration Length')
legend('Inferred', 'Naive')

errorbar(segment{1}.nData(1:50:end,1)/nTime, means(1:50:end,2), vars(1:50:end,2), 'k.', 'linewidth',2)
errorbar(segment{1}.nData(1:50:end,1)/nTime, means(1:50:end,4), vars(1:50:end,4), 'k.', 'linewidth',2)

axis([0 1 0 1])

