#ifndef __PID_H__
#define __PID_H__

class Pid {
public:
    void Init(float kp, float ki, float kd, float dt, 
    float max_output = 30000, float min_output = -30000);
    ~Pid() = default;
    void Control(float target,float current_data);
    float Result();
private:
    float kp_;
    float ki_;
    float kd_;
    float last_error_;
    float dt_;
    float sum_error_;
    float result_;
    float max_output_;
    float min_output_;
};

void Pid::Init(float kp, float ki, float kd, float dt, float max_output, float min_output) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    dt_ = dt;
    max_output_ = max_output;
    min_output_ = min_output;
}

float Pid::Result(){

    return result_;
}

void Pid::Control(float target, float current_data) {
    float error = target - current_data;
    
    // 比例项
    float p_term = kp_ * error;
    
    // 积分项（带抗饱和）
    float i_term = ki_ * sum_error_ * dt_;
    
    // 计算未限幅的输出
    float output = p_term + i_term + kd_ * (error - last_error_) / dt_;
    
    // 输出限幅
    if (output > max_output_) {
        output = max_output_;
        // 不累积积分项当输出饱和时
    } else if (output < min_output_) {
        output = min_output_;
        // 不累积积分项当输出饱和时
    } else {
        // 只有输出未饱和时才累积积分
        sum_error_ += error;
    }
    
    result_ = output;
    last_error_ = error;
}

#endif