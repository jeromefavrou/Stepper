#ifndef STEPPER_HPP
#define STEPPER_HPP

#ifndef nullptr
  #define nullptr 0x00
#endif

#ifndef Stepper_version
  #define Stepper_version 0x01
#endif

class Stepper
{
  public:
  
    enum DIRECTION: int{BACKWARD=-1,FORWARD=1};
    enum MOVE_TYPE: int{FULL_STEP=0,HIGH_TORQUE,HALF_STEP};
    
    Stepper(unsigned long const  Nb_step, unsigned long const Reduct_factor=1):m_nb_step(Nb_step*Reduct_factor)
    {
      this->m_inter_delay=0;
      this->m_dir=1;
      this->m_max_freq=100;
      this->m_min_freq=0;
      this->m_freq=0;
      this->m_elaps=micros();
      this->m_enable_pin=nullptr;
      this->m_nb_pin_A=0;
      this->m_nb_pin_B=0;
      this->m_pin_A=nullptr;
      this->m_pin_B=nullptr;
      this->m_init=false;
      this->m_stop=true;
      this->m_ie=0;
      this->m_stop_step_pos=9;
    };
    
    virtual ~Stepper(void)
    {
      ///destrucot, free all dynamics variables
      
      for(auto i=0;i<this->m_nb_pin_A;i++)
      {
        digitalWrite(this->m_pin_A[i],LOW);
        this->m_pin_A[i]=0;
      }
      
      for(auto i=0;i<this->m_nb_pin_B;i++)
      {
        digitalWrite(this->m_pin_B[i],LOW);
        this->m_pin_B[i]=0;
      }
      
      if(this->m_pin_A!=nullptr)
        delete [] this->m_pin_A;

      if(this->m_pin_B!=nullptr)
        delete [] this->m_pin_B;

      if(this->m_enable_pin!=nullptr)
      {
        this->m_enable_pin=0;
        delete this->m_enable_pin;
      }
    };
  
    inline void direction(DIRECTION const & _dir)
    {
      ///set direction of motor
      
      if(!this->m_stop)
        return;
        
      this->m_dir=int(_dir)>=0?1:-1;
    }
    
    inline void max_freq(int const & _max_freq)
    {
      ///set max frequences
      this->m_max_freq=_max_freq;
    }
    
    inline void min_freq(int const & _min_freq)
    {
      ///set min frequences
      this->m_min_freq=_min_freq;
    }
    
    inline void speed_freq(float const & _hz)
    {
      ///set speed in hertz , is calculate in microseconde and in Hertz
      this->m_freq=_hz;
      this->m_inter_delay=this->m_max_freq<this->m_freq?(1000000.0/float(this->m_max_freq)):this->m_min_freq>this->m_freq?(1000000.0/float(this->m_min_freq)):1000000.0/_hz;
    }
    
    inline void speed_rpm(float const & _rpm)
    {
      ///set speed in rotate per minute , is calculate in microseconde and in Hertz
      this->m_inter_delay = 60000000.0/_rpm/float(this->m_nb_step);
      this->m_freq=1.0/(float(this->m_inter_delay)/1000000.0);
      this->m_inter_delay=this->m_max_freq<this->m_freq?(1000000.0/float(this->m_max_freq)):this->m_min_freq>this->m_freq?(1000000.0/float(this->m_min_freq)):this->m_inter_delay;
    }
    
    inline void speed_spm(unsigned long const & _spm)
    {
      ///set speed in step per minute , is calculate in microseconde and in Hertz
      this->m_inter_delay = 60000000.0/float(_spm);
      this->m_freq=1.0/(float(this->m_inter_delay)/1000000.0);
      this->m_inter_delay=this->m_max_freq<this->m_freq?(1000000.0/float(this->m_max_freq)):this->m_min_freq>this->m_freq?(1000000.0/float(this->m_min_freq)):this->m_inter_delay;
    }

    inline void enable(bool const & _enable)
    {
      ///set enable pin if used, delay is secure for LD297,298, or other bright ....
      delayMicroseconds(1);
      
      if(this->m_enable_pin!=nullptr)
        digitalWrite(*this->m_enable_pin, _enable);

      delayMicroseconds(1);
    }

    inline void start(void)
    {
      /// set to start for async methode
      if(!this->m_stop)
        return;
      
      this->m_stop=false;
      this->enable(true);
      this->m_ie=0;
      this->m_elaps=micros();
    }

    inline void stop(void)
    {
      /// set to stop for async methode
      if(this->m_stop)
        return;
        
      this->m_stop=true;
      this->enable(false);
      this->n_step(9,LOW,LOW,LOW,LOW);
      this->m_ie=0;
    }

    template<int a_size,int b_size> inline void init(short const  _pin_A[a_size],short const _pin_B[b_size],short const & _enable=-1)
    {
      if(!this->m_stop)
        return ;
        
      ///Pin attributuion
      this->m_nb_pin_A=a_size;
      this->m_nb_pin_B=b_size;

      this->m_pin_A=new short[a_size];
      this->m_pin_B=new short[b_size];

      ///Pin initialize
      for(auto i=0;i<a_size;i++)
      {
        this->m_pin_A[i]=_pin_A[i];
        pinMode(this->m_pin_A[i],OUTPUT);
      }

      for(auto i=0;i<b_size;i++)
      {
        this->m_pin_B[i]=_pin_B[i];
        pinMode(this->m_pin_B[i],OUTPUT);
      }

      ///If use an enable Pin
      if(_enable > -1)
      {
        this->m_enable_pin=new short(_enable);
        pinMode(*this->m_enable_pin,OUTPUT);
      }

      this->n_step(9,LOW,LOW,LOW,LOW);
      
      this->m_init=true;
    }
    
    virtual void move_laps(float const & _nb_laps,MOVE_TYPE const& _mv_type=MOVE_TYPE::FULL_STEP)
    {
      ///rotate for n laps and for an sequance type
      this->move_steps(_nb_laps*this->m_nb_step,_mv_type);
    }
    
    virtual void move_steps(unsigned long const & _nb_steps,MOVE_TYPE const& _mv_type=MOVE_TYPE::FULL_STEP)
    {
      ///rotate for n steps and for an sequance type
      if(!this->m_init || !this->m_stop)
        return ;

      this->m_stop=false;
      
      this->m_ie=0;
      this->m_elaps=micros();
      int step=0;

      this->enable(HIGH);

      while(step< (_mv_type==MOVE_TYPE::HALF_STEP?_nb_steps*2:_nb_steps) )
      {
        unsigned long r_elsp=micros()-this->m_elaps;

        if( (_mv_type==MOVE_TYPE::HALF_STEP?this->m_inter_delay/2.0:this->m_inter_delay)  <=r_elsp-this->m_ie)
        {
          this->next_step(_mv_type);
          this->m_elaps=micros();
          this->m_ie+=this->m_inter_delay-r_elsp;

          step++;
        }
      }

      this->n_step(9,LOW,LOW,LOW,LOW);
      this->enable(LOW);
      
      this->m_ie=0;
      this->m_stop=true;
    }
    
    virtual void move_async(MOVE_TYPE const& _mv_type=MOVE_TYPE::FULL_STEP)
    {
      /// rotate in async in the main or loop and dont stop this
      
      if(!this->m_init || this->m_stop)
      {
        this->m_elaps=micros();
        return ;
      }
      
      ///for this methode stop and start can be external

      unsigned long r_elsp=micros()-this->m_elaps;

      if( (_mv_type==MOVE_TYPE::HALF_STEP?this->m_inter_delay/2.0:this->m_inter_delay) <=r_elsp-this->m_ie)
      {
        this->next_step(_mv_type);
        this->m_elaps=micros();
        ///correct the error in intergate 
        this->m_ie+=this->m_inter_delay-r_elsp;
      }
    }
    
  protected:

  void next_step(MOVE_TYPE const& _mv_type)
  {
    ///this methode determinate the nexte step and the sequance type
    int next=this->m_stop_step_pos+this->m_dir;

    switch(_mv_type)
    {
      /// for full step
      case MOVE_TYPE::FULL_STEP :
      
        next=next<1?4:next>4?1:next;
        switch(next)
        {
          case 1: this->n_step(1,HIGH,LOW,LOW,LOW); break;
          case 2: this->n_step(2,LOW,LOW,HIGH,LOW); break;
          case 3: this->n_step(3,LOW,HIGH,LOW,LOW); break;
          case 4: this->n_step(4,LOW,LOW,LOW,HIGH); break;
          default: return;
        }
      break;

      ///for high torque
      case MOVE_TYPE::HIGH_TORQUE :
      
        next=next<1?4:next>4?1:next;
        switch(next)
        {
          case 1: this->n_step(1,LOW,HIGH,HIGH,LOW); break;
          case 2: this->n_step(2,LOW,HIGH,LOW,HIGH); break;
          case 3: this->n_step(3,HIGH,LOW,LOW,HIGH); break;
          case 4: this->n_step(4,HIGH,LOW,HIGH,LOW); break;
          default: return;
        }
      break;

      ///for half step
      case MOVE_TYPE::HALF_STEP:
      
        next=next<1?8:next>8?1:next;
        switch(next)
        {
          case 1: this->n_step(1,HIGH,LOW,LOW,LOW); break;
          case 2: this->n_step(2,HIGH,LOW,HIGH,LOW); break;
          case 3: this->n_step(3,LOW,LOW,HIGH,LOW); break;
          case 4: this->n_step(4,LOW,HIGH,HIGH,LOW); break;
          case 5: this->n_step(5,LOW,HIGH,LOW,LOW); break;
          case 6: this->n_step(6,LOW,HIGH,LOW,HIGH); break;
          case 7: this->n_step(7,LOW,LOW,LOW,HIGH); break;
          case 8: this->n_step(8,HIGH,LOW,LOW,HIGH); break;
          default: return;
        }
      break;
      default :return;
    }
  }
  void n_step(int const idx_step,bool const a1,bool const a2,bool const b1,bool const b2) 
  {
    ///write digital pin and determine if 2 or 4 pin , (2 pin as 4 pin with invertor of signal for a2 and b2 on the hardware)
    if(m_nb_pin_A>0)
    {
      digitalWrite(this->m_pin_A[0], a1);
      
      if(m_nb_pin_A==2)
        digitalWrite(this->m_pin_A[1], a2);
    }
     
    if(m_nb_pin_B>0)
    {
      digitalWrite(this->m_pin_B[0], b1);
      
      if(m_nb_pin_B==2) 
        digitalWrite(this->m_pin_B[1], b2);   
    }
    
    this->m_stop_step_pos=idx_step;
  }
  
    unsigned long m_nb_step, m_inter_delay, m_elaps;
    int m_dir, m_max_freq, m_min_freq,m_stop_step_pos;
    float m_freq;
    long m_ie;
    bool m_init, m_stop;

    short * m_enable_pin;
    
    short m_nb_pin_A, m_nb_pin_B;
    short * m_pin_A;
    short * m_pin_B;
};

#endif
