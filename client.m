function client(port)
%   provides a menu for accessing PIC32 motor control functions
%
port ='/dev/tty.usbserial-DM00QNU8';

%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.

% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end


fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',10);
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));

has_quit = false;
% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('     a: read ADC current (counts)          b: read ADC current (mA)\n');
    fprintf('     c: read encoder (counts)              d: read encoder (degrees)\n');
    fprintf('     e: reset encoder count                f: set PWM (-100 to 100)\n');
    fprintf('     g: set current gains                  h: get current gains\n');
    fprintf('     i: set position gains                 j: get position gains\n');
    fprintf('     k: test current gains                 l: go to angle (deg)\n');
    fprintf('     m: load step trajectory               n: load cubic trajectory\n');
    fprintf('     o: execute trajectory                 p: unpower the motor\n');
    fprintf('     q: quit                               r: get mode\n');
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');

    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);

    % take the appropriate action
    switch selection
        case 'a'
            adc_counts = fscanf(mySerial, '%d');
            fprintf('The ADC current is %d counts.\n',adc_counts);
        case 'b'
            adc_mA = fscanf(mySerial, '%d');
            fprintf('The ADC current is %d mA.\n',adc_mA);
        case 'c'
            counts = fscanf(mySerial, '%d');
            fprintf('The motor angle is %d counts.\n',counts);
        case 'd'                         % example operation
            degrees = fscanf(mySerial,'%d');   % get the incremented number back
            fprintf('The motor angle is %g degrees\n',degrees/10);     % print it to the screen
        case 'e'
            fprintf('Encoder count successfuly reset.\n');
        case 'f'
            pwm = input('Set PWM (-100 to 100): ');
            fprintf(mySerial,'%d\n',pwm);
            n = fscanf(mySerial, '%d %d');
            switch n(2)
                case 0
                    dir = 'FWD';
                case 1
                    dir = 'REV';
            end
            fprintf('Duty cycle: %d; Direction: %s\n', n(1), dir);
        case 'g'
            set_c_gains = input('Set current controller Kp and Ki as [Kp Ki] ([10 6.5] is recommended): ');
            fprintf(mySerial,'%f %f\n',set_c_gains);
        case 'h'
            get_c_gains = fscanf(mySerial,'%f %f');
            fprintf(['Current: Kp: ',num2str(get_c_gains(1)),', Ki: ',num2str(get_c_gains(2))]);
        case 'i'
            set_p_gains = input('Set position controller Kp, Ki, and Kd as [Kp Ki Kd] ([5 0.05 100] is recommended): ');
            fprintf(mySerial,'%f %f %f\n',set_p_gains);
        case 'j'
            get_p_gains = fscanf(mySerial,'%f %f');
            fprintf(['Position: Kp: ',num2str(get_p_gains(1)),', Ki: ',num2str(get_p_gains(2)),', Kd: ',num2str(get_p_gains(3))]);
        case 'k'
            data = read_plot_matrix(mySerial,'ITEST');
        case 'l'
           go_to_angle = input('Go to angle (deg): ');
           fprintf(mySerial,'%d\n',go_to_angle);
        case 'm'
            reflist = input('Enter a Nx2 matrix of via points [time1, pos1; time2, pos2,...]');
            duration = ceil(max(reflist(:,1)));
            if duration > (10 - (1/200))
                msg = 'maximum trajectory time is 10 seconds.';
                error(msg);
            end
            fprintf('Plotting the desired trajectory and sending to the PIC32 ...\n')
            ref = genRef(reflist,'step');
            fprintf(mySerial,'%d\n',length(ref));   % tell the PIC32 how many data points to expect
            for i=1:length(ref)
               fprintf(mySerial,'%d\n',floor(ref(i)*10));    % send positions to PIC32, in 1/10ths of degrees
            end
        case 'n'
            reflist = input('Enter a Nx2 matrix of via points [time1, pos1; time2, pos2,...]');
            duration = ceil(max(reflist(:,1)));
            if duration > (10 - (1/200))
                msg = 'maximum trajectory time is 10 seconds.';
                error(msg);
            end
            fprintf('Plotting the desired trajectory and sending to the PIC32 ...\n')
            ref = genRef(reflist,'cubic');
            fprintf(mySerial,'%d\n',length(ref));
            % tell the PIC32 how many data points to expect
            for i=1:length(ref)
               fprintf(mySerial,'%d\n',floor(ref(i)*10));    % send positions to PIC32, in 1/10ths of degrees
            end
        case 'o'
            data = read_plot_matrix(mySerial,'TRACK');
        case 'p'
            fprintf('Motor OFF.\n');
        case 'q'
            has_quit = true;             % exit client
        case 'r'
            n = fscanf(mySerial, '%d');
            switch n
                case 0
                    mode = 'IDLE';
                case 1
                    mode = 'PWM';
                case 2
                    mode = 'ITEST';
                case 3
                    mode = 'HOLD';
                case 4
                    mode = 'TRACK';
                otherwise
                    msg = 'unknown mode';
                    error(msg);
            end
            fprintf('Mode: %s\n', mode);
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
    fprintf('\n');
end
end
