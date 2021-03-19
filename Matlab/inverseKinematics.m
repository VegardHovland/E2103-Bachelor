physicalProportions;

% Desired configuration

phi     = pi/4;


d       = [hipLength 0 0 0 0]; 
theta   = [0 0 0 0 phalangesAngle];
a       = [hipHeigth femurLength tibiaLength tarsalLength sphalangesLength-24];
alpha   = [hipRotation 0 0 0 0];  

rp = sqrt(xRobot^2+yRobot^2);
rn = -rp;
s = zRobot-d(1);

theta1 = pi + atan2(y,x);

rm = rp-a(1)-a(4)*cos(phi-theta(5))-a(5)*cos(phi);
sm = s-a(4)*sin(phi-theta(5))-a(5)*sin(phi);

gamma = atan2(-sm/sqrt(rm^2+sm^2),-rm/sqrt(rm^2+sm^2));
theta(2) = gamma + acos(-(rm^2+sm^2+a(2)^2-a(3)^2)/(2*a(2)*sqrt((-2*rm*a(2))^2+(-2*sm*a(2))^2)));
theta(3) = atan2((sm-a(2)*sin(theta(2)))/a(3),(rm-a(2)*cos(theta(2)))/a(3));
theta(4) = phi-(theta(2)+theta(3));

