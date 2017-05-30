#ifndef _PID_H_
#define _PID_H_


typedef struct
{
    //PID ��������
    float Kp;
    float Ki;
    float Kd;
    //���ַ���
    float beta;//0���ǲ������ַ��룬 �����Ϊ����ô���ǻ��ַ�Χ
    //΢������
    int dif_prior;//0�ǲ���΢������
    //���ٻ���
    //PID���ֵ
    float last_U; //��һ�ε����ֵ
    float delta_U;//����ʽPID
    float U;//λ��ʽPID
    //�趨ֵ
    float s[3];
    //���
    float e[3];//ÿ�ε����2�����µ�1����һ�ε�0�Ǵ��ϴ�
    //ʵ��ֵ
    float r[3];
    //ǰ������
    float Kvff;
    float Kaff;
    float integral;
} PidTypeDef;

typedef struct
{
    float Kp_C;
    float Ki_C;
    float Kd_C;
    float s_C[3];
    float e_C[3];//ÿ�ε����2�����µ�1����һ�ε�0�Ǵ��ϴ�
    //ʵ��ֵ
    float r_C[3];
    float last_U_C; //��һ�ε����ֵ
    float delta_U_C;//����ʽPID
    float U_C;//λ��ʽPID
} PidCurrentTypeDef;

typedef struct
{
    float w11;
    float w12;
    float w21;
    float w22;
    float w31;
    float w32;
    float q1[2];
    float q2[2];
    float q3[2]; //�����������Ԫ���ֱ������������֣�΢�ֲ���
    float x1[2]; //
    float x2[2];//
    float x3[2];//��������
    float out;
} nn_PidTypeDef;

void PID_Init(PidTypeDef * pid);

void PID_SetParam(PidTypeDef * pid, float p, float i, float d, float beta,
                  float dif_prior, float kaff, float kvff);
void PID_CurrentParam(PidCurrentTypeDef * pid, float p, float i, float d);

void PID_Calc(PidTypeDef * pid, float set_val, float rel_val, int32_t dertT);
void PID_CurrentCalc(PidCurrentTypeDef * pid, float rel_val, float set_val);

#endif
