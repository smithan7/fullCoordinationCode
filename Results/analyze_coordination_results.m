close all
clear all
clc

% load file

com_ranges = [100,90,80,70,60,50,40,30,20,10,5];

fName1 = '/home/rdml/coordination_testing/fullCoordinationCode/Results/Coordination_gmapping_los_coms_';
fName2 = '_all_methods.csv';

for iters = 1:length(com_ranges)
    
    filename = strcat(fName1, num2str(com_ranges(iters)), fName2);
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

    % get the time required to explore each segment by exploration type
    mf_cntr =1; np_cntr =1; ip_cntr=1;

    for i=1:length(segment(1,:))
        if strcmp(segment{i}.coordination,'marketFrontiers')
            times.marketFrontier(mf_cntr) = segment{i}.data(end,1);
            mf_cntr = mf_cntr+1;
        else if strcmp(segment{i}.coordination,'selectPose') && strcmp(segment{i}.inference,'naive')
                times.naivePose(np_cntr) = segment{i}.data(end,1);
                np_cntr = np_cntr+1;
            else if strcmp(segment{i}.coordination,'selectPose')
                    times.inferredPose(ip_cntr) = segment{i}.data(end,1);
                    ip_cntr = ip_cntr +1;
                end
            end
        end 
    end
    
    times.mean_mf(iters) = mean( times.marketFrontier );
    times.mean_np(iters) = mean( times.naivePose );
    times.mean_ip(iters) = mean( times.inferredPose );
    
    times.var_mf(iters) = std( times.marketFrontier )/sqrt(length(times.marketFrontier));
    times.var_np(iters) = std( times.naivePose )/sqrt(length(times.marketFrontier));
    times.var_ip(iters) = std( times.inferredPose )/sqrt(length(times.marketFrontier));
end

figure
hold all
plot(com_ranges, times.mean_mf, 'r', 'linewidth',2)
plot(com_ranges, times.mean_np, 'g', 'linewidth',2)
plot(com_ranges, times.mean_ip, 'b', 'linewidth',2)

xlabel('Communication Range')
ylabel('Exploration Length')
legend('Frontier','Naive Pose', 'Inferred Pose')

errorbar(com_ranges, times.mean_mf, times.var_mf, 'k.', 'linewidth',2)
errorbar(com_ranges, times.mean_np, times.var_np, 'k.', 'linewidth',2)
errorbar(com_ranges, times.mean_ip, times.var_ip, 'k.', 'linewidth',2)
