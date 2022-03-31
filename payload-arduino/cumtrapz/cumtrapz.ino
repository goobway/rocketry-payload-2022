void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
/*
      xVel = zeros(length(xAcc),1);
      xDis = zeros(length(xAcc),1);
      
      for i = 1:length(timeStep)-1
        b = xAcc(i+1);
        a = xAcc(i);
        xVel(i) = (b+a)/2*tstep;
      end
      
      xVelCur = zeros(length(xVel),1);
      for i = 2:length(xVelCur)
        xVelCur(i) = xVelCur(i-1) + xVel(i);
      end
      
      for i = 1:length(timeStep)-1
        b = xVelCur(i+1);
        a = xVelCur(i);
        xDis(i) = (b+a)/2*tstep;
      end
      
      xDisTot = sum(xDis)
      
      xDisCur = zeros(length(xDis),1);
      for i = 2:length(xDisCur)
        xDisCur(i) = xDisCur(i-1) + xDis(i);
      end
*/
