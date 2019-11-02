function data = read_plot_matrix(mySerial,mode)
    nsamples = fscanf(mySerial,'%d');           % first get the number of samples being sent
    data = zeros(nsamples,2);                   % two values per sample: ref and actual
    for i=1:nsamples
       data(i,:) = fscanf(mySerial,'%d %d');    % read in data from PIC32; assume ints, in mA
       times(i) = (i-1)*0.2;                    % 0.2 ms between samples
    end
    if nsamples > 1
        stairs(times,data(:,1:2));              % plot the reference and actual
    else
        fprintf('Only 1 sample received\n')
        disp(data);
    end
    % compute the average error
    score = mean(abs(data(:,1)-data(:,2)));
    switch mode
        case 'ITEST'
            fprintf('\nAverage error: %5.1f mA\n',score);
            title(sprintf('Average error: %5.1f mA\n',score));
            ylabel('Current (mA)');
            xlabel('Time (ms)');
        case 'TRACK'
            fprintf('\nAverage error: %5.1f deg\n',score/10);
            title(sprintf('Average error: %5.1f deg\n',score/10));
            ylabel('Position (deg)');
            xlabel('Time (ms)');
    end
    legend('Reference','Actual','Location','Best')
end