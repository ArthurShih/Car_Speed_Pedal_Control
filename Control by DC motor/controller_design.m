close all;
%% P
figure(1);
rlocus(tf_sys);
%% PI
% positve zero
% Adjust gain
s = tf("s");
PI_1 = (s-4)/s;
figure(2);
subplot(2,1,1);
for K_pi = -0.1:-0.1:-0.5
    pzmap(feedback(K_pi*PI_1*tf_sys,1));
    hold on;
end
hold off;
legend("0.1","0.2","0.3","0.4","0.5");
subtitle("a = 4");
subplot(2,1,2);
for K_pi = -0.1:-0.1:-0.5
    step(feedback(K_pi*PI_1*tf_sys,1));
    hold on;
end
hold off;
legend("0.1","0.2","0.3","0.4","0.5");
subtitle("a = 4");
% Adjust zero
figure(3);
subplot(2,1,1);
for a = 1:5
    PI_2 = -0.25*(s-a)/s;
    pzmap(feedback(PI_2*tf_sys,1));
    hold on;
end
hold off;
legend("1","2","3","4","5");
subtitle("Kp = -0.25");
subplot(2,1,2);
for a = 1:5
    PI_2 = -0.25*(s-a)/s;
    step(feedback(PI_2*tf_sys,1));
    hold on;
end
step(feedback((-0.25*(s-4)/s)*tf_sys,1));
hold off;
legend("1","2","3","4","5");
subtitle("Kp = -0.25");

% negative zero
% Adjust gain
figure(4);
subplot(2,1,1);
PI_3 = (s+4)/s;
for K_pi = 0.1:0.1:0.5
    pzmap(feedback(K_pi*PI_3*tf_sys,1));
    hold on;
end
hold off;
legend("0.1","0.2","0.3","0.4","0.5");
subtitle("a = -4");
subplot(2,1,2);
for K_pi = 0.1:0.1:0.5
    step(feedback(K_pi*PI_3*tf_sys,1));
    hold on;
end
hold off;
legend("0.1","0.2","0.3","0.4","0.5");
subtitle("a = -4");
% Adjust zero
figure(5);
subplot(2,1,1);
for a = 2:2:8
    PI_4 = 0.4*(s+a)/s;
    pzmap(feedback(PI_4*tf_sys,1));
    hold on;
end
hold off;
legend("2","4","6","8");
subtitle("Kp = 0.4");
subplot(2,1,2);
for a = 2:2:8
    PI_4 = 0.4*(s+a)/s;
    step(feedback(PI_4*tf_sys,1));
    hold on;
end
hold off;
legend("2","4","6","8");
subtitle("Kp = 0.4");
%% PID
figure(6)
for a = -2:-1:-4
    for b = -2:-1:-4
        PID = (s-a)*(s-b)/s;
        step(feedback(PID*tf_sys,1));
        hold on;
    end
end
legend("-2,-2","-2,-3","-2,-4","-3,-2","-3,-3","-3,-4","-4,-2","-4,-3","-4,-4")
hold off;
figure(7)
for K = 1:0.2:3
    PID_2 = K*(s+3)*(s+2)/s;
    step(feedback(PID_2*tf_sys,1));
    hold on;
end
legend("1","1.2","1.4","1.6","1.8","2","2.2","2.4","2.6","2.8","3");
hold off;