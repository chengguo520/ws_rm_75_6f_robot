#include<iostream>
#include "linux-x86/include/rm_base.h"


//////////////////////////////////////////////////////////////////////////////////
//睿尔曼智能科技有限公司        Author:Chen Shudong 
//创建日期:2022/05/16
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 睿尔曼智能科技有限公司
//All rights reserved
//文档说明：Cmake下使用睿尔曼API接口示例
//////////////////////////////////////////////////////////////////////////////////

#define RMERR_SUCC  0
#define ARM_DOF     7

// 手动维护句柄
SOCKHANDLE m_sockhand = -1;

// 休眠(毫秒)
void sleep_cp(int milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds);
#endif

#ifdef __linux
    usleep(milliseconds * 1000);
#endif
}

// 透传接口回调函数
void MCallback(CallbackData data)
{
    // 判断接口类型
    switch(data.errCode)
    {
        case MOVEJ_CANFD_CB: // 角度透传
            printf("MOVEJ_CANFD 透传结果: %d\r\n", data.errCode);
        break;
        case MOVEP_CANFD_CB: // 位姿透传
            printf("MOVEP_CANFD  透传结果: %d\r\n", data.errCode);
        break;
        case FORCE_POSITION_MOVE_CB: // 力位混合透传
            printf("FORCE_POSITION_MOVE  透传结果: %d\r\n", data.errCode);
        break;
    }

}

//  常用接口测试
void FuncTest()
{
    int ret = -1;
    // 网络连接状态
    ret = Arm_Sockrt_State(m_sockhand);
    printf("Arm_Sockrt_State ret:%d\r\n", ret);

    // 设置关节零位
    ret = Set_Joint_Zero_Pos(m_sockhand, 1, 1);
    printf("Set_Joint_Zero_Pos ret:%d\r\n", ret);

    // 设置关节最大速度
    ret = Set_Joint_Speed(m_sockhand, 1, 10, 1);
    printf("Set_Joint_Speed ret:%d\r\n", ret);

    // 设置关节最小限位;
    ret = Set_Joint_Min_Pos(m_sockhand, 1, -180, 1);
    printf("Set_Joint_Min_Pos ret:%d\r\n", ret);

    // 设置关节最大限位;
    ret = Set_Joint_Max_Pos(m_sockhand, 1, 180, 1);
    printf("Set_Joint_Max_Pos ret:%d\r\n", ret);

    // 设置关节最大加速度
    ret = Set_Joint_Acc(m_sockhand, 1, 500,1);
    printf("Set_Joint_Acc ret:%d\r\n", ret);

    // 设置关节使能状态
    ret = Set_Joint_EN_State(m_sockhand, 1, 1, 1);
    printf("Set_Joint_EN_State ret:%d\r\n", ret);

    //清除关节错误代码
    ret = Set_Joint_Err_Clear( m_sockhand, 1, 1);
    printf("Set_Joint_Err_Clear ret:%d\r\n", ret);

    //获取关节最大速度
    float fSpeeds[6] = {0,1,2,3,4,5};
    ret = Get_Joint_Speed(m_sockhand,fSpeeds);
    printf("Get_Joint_Speed ret:%d\r\n", ret);

    //获取关节最大加速度
    float fAccs[6] = {0,1,2,3,4,5};
    ret = Get_Joint_Acc(m_sockhand,fAccs);
    printf("Get_Joint_Acc ret:%d\r\n", ret);

    //获取关节最小限位
    float fMinJoint[7] = {0,1,2,3,4,5,6};
    ret = Get_Joint_Min_Pos(m_sockhand,fMinJoint);
    printf("Get_Joint_Min_Pos ret:%d\r\n", ret);

    //获取关节最大限位
    float fMaxJoint[7] = {0,1,2,3,4,5,6};
    ret = Get_Joint_Max_Pos(m_sockhand,fMaxJoint);
    printf("Get_Joint_Max_Pos ret:%d\r\n", ret);

    //设置机械臂末端最大角速度
    float fASpeed = 0.2;
    ret = Set_Arm_Angular_Speed(m_sockhand,fASpeed, 1);
    printf("Set_Arm_Angular_Speed ret:%d\r\n", ret);

    //设置机械臂末端最大角加速度
    float fAAcc = 4;
    ret = Set_Arm_Angular_Acc(m_sockhand,fAAcc, 1);
    printf("Set_Arm_Angular_Acc ret:%d\r\n", ret);

    //获取机械臂末端最大线速度
    float fLSpeed = 0;
    ret = Get_Arm_Line_Speed(m_sockhand,&fLSpeed);
    printf("Get_Arm_Line_Speed ret:%d\r\n", ret);

    //获取机械臂末端最大线加速度
    float fLAcc = 0;
    ret = Get_Arm_Line_Acc(m_sockhand,&fLAcc);
    printf("Get_Arm_Line_Acc ret:%d\r\n", ret);
}

// 画8字
void demo01()
{
    int ret = -1;
    // 回零位
    float joint[6] = {0};
    ret = Movej_Cmd(m_sockhand, joint, 20, 0, 1);
    if(ret != 0)
    {
        printf("Movej_Cmd 1:%d\r\n",ret);
        return;
    }

    // 移动到初始点位
    float joint1[6] = {18.44,-10.677,-124.158,-15,-45.131,-71.445};
    ret = Movej_Cmd(m_sockhand,joint1,20,0,1);
    if(ret != 0)
    {
        printf("Movej_Cmd 2:%d\r\n",ret);
        return;
    }

    // 画8字
    for(int i=0;i<3;i++)
    {
        POSE po1,po2,po3;
        po1.px = 0.186350; po1.py = 0.062099; po1.pz = 0.2; po1.rx = 3.141; po1.ry = 0; po1.rz = 1.569;
        po2.px = 0.21674; po2.py = 0.0925; po2.pz = 0.2; po2.rx = 3.141; po2.ry = 0; po2.rz = 1.569;
        po3.px = 0.20785; po3.py = 0.114; po3.pz = 0.2; po3.rx = 3.141; po3.ry = 0; po3.rz = 1.569;
        ret = Movel_Cmd(m_sockhand, po1, 20, 0, 1);
        if(ret != 0)
        {
            printf("Movel_Cmd 1:%d\r\n",ret);
            return;
        }

        ret = Movec_Cmd(m_sockhand,po2,po3,20,0,0,1);
        if(ret != 0)
        {
            printf("Movec_Cmd 1:%d\r\n",ret);
            return;
        }

        POSE po4 ,po5,po6;
        po4.px = 0.164850;po4.py = 0.157;po4.pz = 0.2;po4.rx = 3.141; po4.ry = 0;po4.rz = 1.569;
        po5.px = 0.186350;po5.py = 0.208889;po5.pz = 0.2; po5.rx = 3.141;po5.ry = 0; po5.rz = 1.569;
        po6.px = 0.20785;po6.py = 0.157;po6.pz = 0.2; po6.rx = 3.141;po6.ry = 0; po6.rz = 1.569;
        ret = Movel_Cmd(m_sockhand,po4,20,0,1);
        if(ret != 0)
        {
            printf("Movel_Cmd 2:%d\r\n",ret);
            return;
        }

        ret = Movec_Cmd(m_sockhand,po5,po6,20,0,0,1);
        if(ret != 0)
        {
            printf("Movec_Cmd 2:%d\r\n",ret);
            return;
        }
    }
    printf("demo01 success:%d\r\n",ret);
}

// 夹爪使用例程
void demo02()
{
    int ret = -1;
    float joint[6]={0,0,0,0,0,0};
    float joint1[6]= {4.61,93.551,75.276,-10.098,-76.508,57.224};
    float joint2[6]= {4.61,67.175,96.152,-10.385,-71.095,58.244};
    float joint3[6]= {-106.244,67.172,96.15,-10.385,-71.097,58.243};

    //  回零位
    ret = Movej_Cmd(m_sockhand,joint,20,0,1);
    if(ret != 0)
    {
        printf("Movej_Cmd 1:%d\r\n",ret);
        return;
    }

    // 张开夹爪，到达抓取位置
    ret = Set_Gripper_Release(m_sockhand,500,1);
    if(ret != 0)
    {
        printf("Set_Gripper_Release 1:%d\r\n",ret);
        return;
    }
    ret = Movej_Cmd(m_sockhand,joint1,20,0,1);
    if(ret != 0)
    {
        printf("Movej_Cmd 2:%d\r\n",ret);
        return;
    }

    // 抓取
    ret = Set_Gripper_Pick_On(m_sockhand,500,500,1);
    if(ret != 0)
    {
        printf("Set_Gripper_Pick_On ret:%d\r\n",ret);
        return;
    }

    // 放置
    ret = Movej_Cmd(m_sockhand,joint3,20,0,1);
    if(ret != 0)
    {
        printf("Movej_Cmd 3:%d\r\n",ret);
        return;
    }
    ret = Set_Gripper_Release(m_sockhand,500,1);
    if(ret != 0)
    {
        printf("Set_Gripper_Release 2:%d\r\n",ret);
        return;
    }
    sleep_cp(200);
    ret = Movej_Cmd(m_sockhand,joint2,20,0,1);
    if(ret != 0)
    {
        printf("Movej_Cmd 4:%d\r\n",ret);
        return;
    }
    printf("demo02 success:%d\r\n",ret);
}

// 力控使用例程
void demo03()
{
    int ret = -1;
    POSE po1,po2;
    float joint[6] = {0.008,83.847,41.590,-0.024,64.588,0.017};
    po1.px = -0.40054; po1.py = 0; po1.pz = 4.360; po1.rx = 3.141;po1.ry = -174;po1.rz = 0;
    po2.px = -0.25305; po2.py = 0; po2.pz = 4.369; po2.rx = 3.141;po2.ry = -174;po2.rz = 0;

    // 初始位置
    ret = Movej_Cmd(m_sockhand,joint,20,0,1);
    if(ret != 0)
    {
        printf("Movej_Cmd 1:%d\r\n",ret);
        return;
    }

    //开启力位混合控制
    ret = Set_Force_Postion(m_sockhand,2,2,1);
    if(ret != 0)
    {
        printf("Set_Force_Postion :%d\r\n",ret);
        return;
    }

    for(int i = 0; i<20; i++)
    {
        ret = Movel_Cmd(m_sockhand,po1,20,0,1);
        ret = Movel_Cmd(m_sockhand,po2,20,0,1);
    }

    if(ret != 0)
    {
        printf("Movel_Cmd :%d\r\n",ret);
        return;
    }
    //结束力控
    ret = Stop_Force_Postion(m_sockhand,1);
    if(ret != 0)
    {
        printf("Stop_Force_Postion :%d\r\n",ret);
        return;
    }
    printf("demo03 success :%d\r\n",ret);
}

// 获取机械臂状态
void demo04()
{
    int ret = -1;
    // 获取工具坐标系
    FRAME tool;
    ret = Get_Current_Tool_Frame(m_sockhand,&tool);
    if(ret != 0)
    {
        printf("Get_Current_Tool_Frame ret:[%s]\r\n",ret);
        return;
    }
    printf("工具坐标系名称:[%s]\r\n", tool.frame_name.name);
    printf("工具姿态:.2f%, .2f%, .2f%, .2f%, .2f%, .2f%\r\n", tool.pose.px, tool.pose.py, tool.pose.pz, tool.pose.rx, tool.pose.ry, tool.pose.rz);
    printf("重量:[%s]\r\n", tool.payload);
    printf("质心:.2f%, .2f%, .2f%\r\n",tool.x, tool.y, tool.z);

    // 获取机械臂当前状态
    float joint[6] ; POSE po1; uint16_t Arm_Err; uint16_t Sys_Err;
    ret = Get_Current_Arm_State(m_sockhand,joint,&po1,&Arm_Err,&Sys_Err);
    if(ret != 0)
    {
        printf("Get_Current_Arm_State ret:%d\r\n",ret);
        return;
    }
    printf("关节角度:.2f%, .2f%, .2f% .2f%, .2f%, .2f%\r\n",joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
    printf("末端位姿:.2f%, .2f%, .2f%, .2f%, .2f%, .2f%\r\n",po1.px, po1.py, po1.pz, po1.rx, po1.ry, po1.rz);
    printf("机械臂运行错误代码:[%s]\r\n",Arm_Err);
    printf("控制器错误代码:[%s]\r\n",Sys_Err);

    // 关节角度运动
    float fl[6] = {0,0,0,0,90,0};
    ret = Movej_Cmd(m_sockhand,fl,20,0,1);
    if(ret != 0)
    {
        printf("Movej_Cmd :%d\r\n",ret);
        return;
    }

    // 获取机械臂当前状态
    ret = Get_Current_Arm_State(m_sockhand,joint,&po1,&Arm_Err,&Sys_Err);
    if(ret != 0)
    {
        printf("Get_Current_Arm_State ret:%d\r\n",ret);
        return;
    }
    printf("关节角度:.2f%, .2f%, .2f% .2f%, .2f%, .2f%\r\n",joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
    printf("末端位姿:.2f%, .2f%, .2f%, .2f%, .2f%, .2f%\r\n",po1.px, po1.py, po1.pz, po1.rx, po1.ry, po1.rz);
    printf("机械臂运行错误代码:[%s]\r\n",Arm_Err);
    printf("控制器错误代码:[%s]\r\n",Sys_Err);
    printf("demo04 success :%d\r\n",ret);
}

// 关节示教
void demo05()
{
    int ret = -1;

    // 示教
    byte num = 2; byte direction = 1; byte v = 20;
    ret = Joint_Teach_Cmd(m_sockhand,num ,direction ,v,1);

    if(ret != 0)
    {
        printf("Joint_Teach_Cmd ret:%d\r\n",ret);
        return;
    }
    sleep_cp(2000);

    ret = Teach_Stop_Cmd(m_sockhand,1);
    if(ret != 0)
    {
        printf("Teach_Stop_Cmd ret:%d\r\n",ret);
        return;
    }

    printf("demo05 success :%d\r\n",ret);
}

// 角度透传
void demo06()
{
    int ret = -1;

    // 获取机械臂当前状态
    float joint[6] ; POSE po1; uint16_t Arm_Err; uint16_t Sys_Err;
    ret = Get_Current_Arm_State(m_sockhand,joint,&po1,&Arm_Err,&Sys_Err);
    if(ret != 0)
    {
        printf("Get_Current_Arm_State ret:%d\r\n",ret);
        return;
    }
    printf("关节角度:.2f%, .2f%, .2f% .2f%, .2f%, .2f%\r\n",joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
    printf("末端位姿:.2f%, .2f%, .2f%, .2f%, .2f%, .2f%\r\n",po1.px, po1.py, po1.pz, po1.rx, po1.ry, po1.rz);
    printf("机械臂运行错误代码:[%s]\r\n",Arm_Err);
    printf("控制器错误代码:[%s]\r\n",Sys_Err);

    for(int i = 1;i<=100;i++)
    {
        joint[5] += 0.4;
        ret = Movej_CANFD(m_sockhand, joint);
    }

    if (ret != 0)
    {
        printf("Movej_CANFD ret: %d\r\n",ret);
    }
    printf("demo06 success :%d\r\n",ret);
}


using namespace std;
int main(){
    // 连接
    // 初始化API, 注册回调函数
    RM_API_Init(MCallback);
    // 连接服务器
    m_sockhand =  Arm_Socket_Start((char*)"192.168.1.209", 8080, ARM_65, 5000);

    // 常用接口测试
    FuncTest();

    // 关闭连接
    Arm_Socket_Close(m_sockhand);
    m_sockhand = -1;
    return 0;
} 
