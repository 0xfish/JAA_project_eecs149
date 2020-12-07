#define PID_MAX_INTEGRAL         2000
#define ZUMO_BASE_DEADBAND       0

typedef struct PIDLoop {
  int32_t m_command; 

  int32_t m_pgain;
  int32_t m_igain;
  int32_t m_dgain;
  int32_t m_min;
  int32_t m_max;
  int32_t m_deadband;
  bool    m_servo;
  
  int32_t m_prevError;
  int32_t m_integral;
} pid_loop_t;


void pid_init(pid_loop_t *pid, int32_t p, int32_t i, int32_t d, bool servo);
void pid_reset(pid_loop_t *pid);
void pid_update(pid_loop_t *pid, int32_t error);


void pid_init(pid_loop_t *p, int32_t pgain, int32_t igain, int32_t dgain, bool servo) {
    p->m_pgain = pgain;
    p->m_igain = igain;
    p->m_dgain = dgain;
    p->m_servo = servo;

    // clamp
    p->m_min = INT32_MIN;
    p->m_max = INT32_MAX;

    pid_reset(p);
}

void pid_reset(pid_loop_t *p) {
    p->m_command = 0;    
    p->m_integral = 0;
    p->m_prevError = 0x80000000L;
}  

void pid_update(pid_loop_t *p, int32_t error) {
    int32_t pid;

    if (p->m_prevError != 0x80000000L) { 
        // integrate integral
        p->m_integral += error;
        // clamp the integral
        if (p->m_integral > PID_MAX_INTEGRAL)
            p->m_integral = PID_MAX_INTEGRAL;
        else if (p->m_integral < -PID_MAX_INTEGRAL)
            p->m_integral = -PID_MAX_INTEGRAL;

        // calculate PID term
        pid = error*p->m_pgain + (p->m_integral*p->m_igain) + (error - p->m_prevError)*p->m_dgain;

        if (p->m_servo) {
            p->m_command += pid; // since servo is a position device, we integrate the pid term
            if (p->m_command < p->m_min) 
                p->m_command = p->m_min; 
            else if (p->m_command > p->m_max) 
                p->m_command = p->m_max;
        } else {
            // Deal with Zumo base deadband
            if (pid > 0)
                pid += ZUMO_BASE_DEADBAND;
            else if (pid < 0)
                pid -= ZUMO_BASE_DEADBAND;
            p->m_command = pid; // Zumo base is velocity device, use the pid term directly  
        }
    }

    // retain the previous error val so we can calc the derivative
    p->m_prevError = error; 
}
