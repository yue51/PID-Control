class PIDController:
    def __init__(self, target_pos):
        self.target_pos = target_pos
        self.Kp = 5212.78#2563.03 
        self.Ki = 4276.36#2878.15
        self.Kd =1155.46 #1029# 338.15
        self.bias = 0.0
        self.error_prev = 0.0
        self.error_cur = 0.0
        self.dt = 1.0/60.0
        self.integral_error = 0.0
        
        return

    def reset(self):
        self.integral_error = 0.0
        self.error_prev = 0.0
        return

#TODO: Complete your PID control within this function. At the moment, it holds
#      only the bias. Your final solution must use the error between the 
#      target_pos and the ball position, plus the PID gains. You cannot
#      use the bias in your final answer. 
    def get_fan_rpm(self, vertical_ball_position):
       # output = self.bias
        error = self.target_pos - vertical_ball_position
        self.integral_error += error*self.dt
        self.error_cur = (error - self.error_prev) /self.dt
        #
        

        #calculate pid:  u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        P = self.Kp * error
        I = self.Ki * self.integral_error
       # if(I < -threshold):
        #    I = -threshold
      #  elif(I >threshold):
      #      I = threshold
        D = self.Kd * self.error_cur
        if self.error_prev < 0:
            output = P+I
        else:
            output = P+I+D
        self.error_prev = error  
        return output 
  