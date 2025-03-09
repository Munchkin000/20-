/*说明:本文件包含了左右圆环的各个标志位的判断函数(第一角点，切点，第二角点，出环角点，出环直行角点)，没有
补线的操作
 * circle.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "zf_common_headfile.h"

#define loop_forward_far  100
#define loop_forward_near 25

//extern int element_number;
//extern int16 element_timer_temp;

/*****************************************************************************
***@breif	入环时检测左环第一个角点(Inloop=1)
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_first_angle()
{
    if(watch.InLoop!=0&&watch.InLoop!=1)//不是左环的入环阶段
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
//        if(//lineinfo[y + 3].left_lost
//            //&&lineinfo[y + 2].left_lost&&
//            lineinfo[y + 1].left_lost&&  //左线上方出现丢线
//			!lineinfo[y - 3].left_lost&&!lineinfo[y - 2].left_lost&&!lineinfo[y - 4].left_lost&&\
//            !lineinfo[y].left_lost&&    //左线下方连续不丢线
//            !lineinfo[y+1].right_lost&&!lineinfo[y].right_lost&&!lineinfo[y-1].right_lost&&//右线连续不丢
//			lineinfo[y].left-lineinfo[y+4].left>10//左线横坐标出现跳变
//            &&y<watch.InLoopAngleL&&y<75)//满足条件行处于合理范围内
            //&&lineinfo[y].left>=lineinfo[y-2].left
		if(((lineinfo[y + 1].left_lost)||  //左线上方出现丢线
			((lineinfo[y].left-lineinfo[y+1].left)>=5*(lineinfo[y+1].left-lineinfo[y+2].left)))
            && !lineinfo[y - 3].left_lost//左线下方连续不丢线
            && !lineinfo[y - 2].left_lost
            && !lineinfo[y - 1].left_lost
            && !lineinfo[y].left_lost
            && !lineinfo[y + 5].right_lost//右线连续不丢
            && !lineinfo[y + 3].right_lost
            && !lineinfo[y + 2].right_lost
            && !lineinfo[y + 1].right_lost
            && !lineinfo[y].right_lost
            && !lineinfo[y - 1].right_lost
            && !lineinfo[y - 2].right_lost
            && !lineinfo[y - 3].right_lost
            && !lineinfo[y - 4].right_lost
            && !lineinfo[y - 5].right_lost
            &&lineinfo[y].left-lineinfo[y+4].left>10 //左线横坐标出现跳变
            &&y<watch.InLoopAngleL
            //&&lineinfo[y].left>=lineinfo[y-2].left
            &&y<75)  //满足条件行处于合理范围内
        {
            watch.InLoopAngleL = y;//记录入左环前直行的第一个角所在行
            if(Element==None) //在当前无元素时进行以下操作，其他时候只找角点
            {
				left_ring_confirm();//进入左环二次确认
            }
            break;//识别到之后退出循环
        }
    }
}
/*****************************************************************************
***@breif	左环入环二次确认函数
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_confirm()
{
	uint8 white_count1=0;//第一角点所在行比角点横坐标小的部分的白点总个数
	uint8 white_count2=0;//第一角点上一行比角点横坐标小的部分的白点总个数
	uint8 white_count3=0;//第一角点上上一行比角点横坐标小的部分的白点总个数
	uint8 black_count=0; //第一角点所在列比角点纵坐标小的部分的黑点总个数
	uint8 right_lost=0;	 //右侧是否丢线标志位
    //for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
	for(int y=loop_forward_near;y<95;y++)//逐行扫描（杜学长的）
    {
//        if((lineinfo[y+2].right-lineinfo[y].right)>3)//右线临近坐标点出现较大变化
//		{
//            right_lost=1;//认为右侧出现丢线
//		}
		if(((lineinfo[y+2].right-lineinfo[y].right)>2)||
			(lineinfo[y].right-lineinfo[y+2].right)>4)//右线临近坐标点出现较大变化(祖传原版)
		{
			right_lost+=1;//认为右侧出现丢线
		}
    }
    if(right_lost<3)//认为右线连续再进行后续判断
    {
		for(int x=lineinfo[watch.InLoopAngleL-1].left;x>0;x--)//遍历左环第一个角点后边的横坐标
		{
			//计算识别到的左环第一角点附近三行的白点总数
			if(Grayscale[119-watch.InLoopAngleL][x]==255)
			{
				white_count1++;
			}
			if(Grayscale[119-(watch.InLoopAngleL-1)][x]==255)
			{
				white_count2++;
			}
			if(Grayscale[119-(watch.InLoopAngleL-2)][x]==255)
			{
				white_count3++;
			}
		}
        for(int y=watch.InLoopAngleL;y>loop_forward_near;y--)//遍历左环第一个角点下边的纵坐标
        {
            if(Grayscale[119-y][lineinfo[watch.InLoopAngleL].left]==0)//统计左环第一角点所在列的黑点总数
			{
               black_count++;
			}
        }			
        if((white_count1>=10&&white_count2>=10&&white_count3>=10)&&(black_count<10))//祖传只有黑色统计
		//横行大量白点，纵列少量黑点
        {			
            enter_element(Left_ring);    //正式进入左圆环元素
            begin_distant_integeral(setpara.GoInLoop_DisIntegral);//开启路程积分(这里要调)
			LEDRED_ON;	//开灯
//			set_speed(setpara.loop_target_speed);
//          change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID(后期提速再说)
			watch.InLoop = 1;	 //检测到左环第一个角点，此时补左侧第一条线保持直行
			return;	 			 //退出检测
/*          else//如果是大环
            {
                  set_speed(setpara.big_loop_speed);
                  //change_pid_para(&CAM_Turn,&setpara.big_loop_PID);
            }*/
         }
    }
	Element=None;//不满足上述条件，认为不是圆环
    watch.InLoopAngleR=120;//初始化第一角点位置
    watch.InLoopAngleL=120;
}
/*****************************************************************************
***@breif	检测左环入环第一个角点后的圆弧
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_circular_arc()
{
    if (watch.InLoop != 1&&watch.InLoop != 2)//不是左环的入环阶段
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
        if (/*//y <watch.InLoopAngle2  &&
             y>watch.InLoopAngleL//圆弧切点在第一角点上方
			 &&y<watch.InLoopCirc//圆弧切点纵坐标值在合理范围内
           &&!lineinfo[y+3].left_lost&&!lineinfo[y+2].left_lost
		   &&!lineinfo[y+1].left_lost&&!lineinfo[y-3].left_lost
           &&!lineinfo[y-2].left_lost&&!lineinfo[y-1].left_lost//左侧不丢线
           &&lineinfo[y+1].left <= lineinfo[y].left&&lineinfo[y+2].left <= lineinfo[y].left
           &&lineinfo[y+3].left <= lineinfo[y].left&&lineinfo[y-1].left <= lineinfo[y].left
		   &&lineinfo[y-2].left <= lineinfo[y].left&&lineinfo[y-3].left <= lineinfo[y].left
           //左侧某点横坐标大于周围点，认为是圆环的圆弧切点*/       
			(watch.InLoopAngleL<65||get_integeral_state(&distance_integral)==2)//圆弧切点在第一角点上方
           &&(y>(watch.InLoopAngleL+20)||get_integeral_state(&distance_integral)==2)//路程积分结束直接进下一状态
           &&y <watch.InLoopCirc//圆弧切点纵坐标值在合理范围内
           &&!lineinfo[y+3].left_lost
           &&!lineinfo[y+2].left_lost
           &&!lineinfo[y+1].left_lost
           &&!lineinfo[y-3].left_lost
           &&!lineinfo[y-2].left_lost
           &&!lineinfo[y-1].left_lost//左侧不丢线
           &&lineinfo[y+1].left <= lineinfo[y].left
           &&lineinfo[y+2].left <= lineinfo[y].left
           &&lineinfo[y+3].left <= lineinfo[y].left
           &&lineinfo[y-1].left <= lineinfo[y].left
           &&lineinfo[y-2].left <= lineinfo[y].left
           &&lineinfo[y-3].left <= lineinfo[y].left//左侧某点横坐标大于周围点，认为是圆环的圆弧切点
		)//下面的是原版祖传
       { 
            watch.InLoopCirc = y;//圆环上凸弧(切点)所在行
            break;
       }
    }
}
/*****************************************************************************
***@breif	检测左环第二个角点(入口角点)
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_second_angle()
{
    if(watch.InLoop != 1&&watch.InLoop != 2)//不是左环的入环阶段
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
//        if (watch.InLoopCirc<120&&//切点处还未被识别到
//            y<watch.InLoopAngle2&&//当前点处于正常范围内
//            watch.InLoopAngle2==120&&//之前第二角点坐标已被清空
//            get_integeral_state(&distance_integral)==2&&//上一段路程积分已完成
//			y < 60&&y <(loop_forward_far-2)&& //当前行在图像上半部分			
//			y>watch.InLoopCirc&&//当前行在圆环切点上方
//           lineinfo[y+1].left > 30&&(lineinfo[y+1].left-lineinfo[y].left)<=2&&//当前行上边连续
//           (lineinfo[y].left-lineinfo[y-4].left)>lineinfo[y].left/3/*当前行下方点出现跳变(丢线)*/)  
		  if (watch.InLoopCirc<66	
			  &&y<watch.InLoopAngle2//当前点处于正常范围内
              &&watch.InLoopAngle2==120//之前第二角点坐标已被清空
              &&get_integeral_state(&distance_integral)==2 //上一段路程积分已完成
			  &&y > 60	//当前行在图像上半部分
              &&y < (loop_forward_far-2)
			  &&y>watch.InLoopCirc//当前行在圆环切点上方
			  &&lineinfo[y+1].left > 30
              &&(lineinfo[y+1].left-lineinfo[y].left)<=2 //当前行上边连续
           &&(lineinfo[y].left-lineinfo[y-4].left)>lineinfo[y].left/2/*当前行下方点出现跳变(丢线)*/
           )//下面是原版祖传		
           {
			   //记录左环第二角点坐标
               watch.InLoopAngle2 = y;
               watch.InLoopAngle2_x=lineinfo[watch.InLoopAngle2].left;
               break;//找到后退出
           }
    }
    if(watch.InLoopAngle2!=120&&watch.InLoopAngle2>50)//左环第二角点纵坐标在图像上边
    {
        find_angle_left_down(watch.InLoopAngle2_x,watch.InLoopAngle2,&watch.InLoopAngle2_x,
		&watch.InLoopAngle2);//向左下找角点不太清除这句干嘛用的
    }
}
/*****************************************************************************
***@breif	左环开始转向函数(watch.InLoop=2)
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_begin_turn()
{
	if(watch.InLoop!=1)//不是入环状态
	{
		return;//在循环之前跳出，节省时间
	}
    if(get_integeral_state(&distance_integral)==2//路程积分完成
        &&watch.InLoop==1	//当前处于未入环阶段
        //&&lineinfo[watch.InLoopCirc].left_lost
        &&watch.InLoopAngle2<=80/*左环第二角点现在离车较近*/)
    {
		watch.InLoop=2;//更改标志位，正在入环，此时右侧补线入环
        clear_distant_integeral();		//清除路程积分变量(准备环内积分)
		change_trailing_para(Left_ring);//将巡线参数调为环内巡线PID（后期提速可能用）
        //watch.fix_slope=(float)(lineinfo[watch.InLoopAngle2].left)/(115-watch.InLoopAngle2);
        begin_angle_integeral(setpara.InLoop_AngleIntegral);//开始圆环入环角度积分
    }
}
/*****************************************************************************
***@breif	检验小车是否完全入环
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_in_loop()
{
    if(watch.InLoop != 2)//现在不是拉线入环阶段
	{
		return;//在循环之前跳出，节省时间
	}
    if( watch.InLoop == 2&&
        get_integeral_state(&angle_integral)==1&&//正在进行圆环内部角度积分
        get_integeral_data(&angle_integral)>40/*转过一定角度*/)
        //&&watch.right_near_lost<40
        //&&lineinfo[watch.right_near_lost].right>180
        //&&(!lineinfo[40].right_lost)        
        {
			watch.InLoop = 3;//更改标志位，此时完全入环
            //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
        }
		
}
/*****************************************************************************
***@breif	小车角度积分快要完成，准备出环
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_prepare_out()
{
    if(watch.InLoop != 3)//小车不是入环状态
	{
		return;//在循环之前跳出，节省时间
	}
    if(watch.InLoop == 3&&
       get_integeral_state(&angle_integral)==1&&//正在进行环内角度积分
       get_integeral_data(&angle_integral)>180 /*积分已经达到一定值*/
	   &&lineinfo[69].right<152
       &&lineinfo[69].right>82)//下面两行是原版祖传
   {
       watch.InLoop = 4;//更改标志位，陀螺仪积分完成，准备出环
       watch.OutLoop_turn_point_x=lineinfo[69].right;//转向点横坐标，根据该点进行补线(为何这么取？)
   }
}
/*****************************************************************************
***@breif	检测出环时右角点位置
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_out_angle()
{
    if(watch.InLoop != 4)//当前不是准备出环状态
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
		if ((watch.InLoop == 4)&&y<80
                 //lineinfo[y].left_lost
             &&lineinfo[y+1].right >= lineinfo[y].right
             &&lineinfo[y+2].right >= lineinfo[y+1].right
             &&lineinfo[y-1].right >= lineinfo[y].right
             &&lineinfo[y-2].right >= lineinfo[y].right//左圆环出环角点的横坐标值小于附近的点
/*                 &&lineinfo[y - 3].right > lineinfo[y - 1].right
                 &&lineinfo[y + 4].right > lineinfo[y + 2].right
                 &&lineinfo[y - 5].right > lineinfo[y - 3].right*/
             &&lineinfo[y].right > 30//角点坐标在图像上边
             &&Grayscale[119-y-2][lineinfo[y].right]==255/*角点上方点为白点*/)

            {
                 if(watch.OutLoopAngle1>y)//之前为检测到角点(清空后角点纵坐标变为120)
                 {
                     //watch.OutLoopRight = lineinfo[y].right;
                     watch.OutLoopAngle1 = y; //记录出环角点纵坐标
                     break;
                 }
            }
    }
}
/*****************************************************************************
***@breif	出左环左转(进入圆环与直线交界处)
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_out_loop_turn()
{
    if(watch.InLoop != 4)//当前不是出环状态
	{
		return;//在循环之前跳出，节省时间
	}
    if(watch.InLoop == 4&&
       //&&(watch.cross+watch.right_lost)>30//右侧丢线过多，说明进入圆环与直线交界处
       watch.OutLoopAngle1<120&&	//检测到出环的角点
       get_integeral_state(&angle_integral)==2//环内角度积分已经完成
       //&&lineinfo[watch.OutLoopAngle1].right_lost==1/*右侧丢线(出环时右边是横着的)*/
		&&watch.OutLoop==0)//还未出环(这句后加的)
    {
        clear_angle_integeral();//清空积分标志位和数值
//        if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
//        {
//            begin_angle_integeral(setpara.loop_angle_out);
//        }
//        else//如果是大环
//        {
//            begin_angle_integeral(setpara.big_loop_out);
//        }				
		begin_angle_integeral(setpara.OutLoop_AngleIntegral);//开始出环角度积分
        begin_distant_integeral(setpara.OutLoop_DisIntegral);//开启路程积分，此时要保持左转(这里需要调)
        watch.OutLoop=1;//出环标志为置1已经出环
    }
}
/*****************************************************************************
***@breif	右侧为直线时直行
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_out_loop()
{
    if(watch.InLoop != 4&&watch.OutLoop!=1)//当前不是出环或出环后状态
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
//        if((watch.InLoop == 4&&watch.OutLoop==1&&
//            y<60&&
//            lineinfo[y].right_lost==0&&lineinfo[y+10].right_lost==0&&
//            (188-(lineinfo[y].right-lineinfo[y+10].right)*(115-y)/10)>40&&//？？？？？
//            //&&(abs(lineinfo[watch.right_near_lost].right-lineinfo[watch.right_near_lost+5].right))<20//右侧为直线
//            //&&(watch.cross+watch.right_lost)<15//右侧丢线数减少
//           get_integeral_state(&distance_integral)==2)/*出环路程积分已完成*/
//           ||get_integeral_state(&angle_integral)==2)
		if((watch.InLoop == 4  //看当前出入环标志位是否正确
            &&watch.OutLoop==1
            &&y<100
            &&lineinfo[y].right_lost==0//右边不再丢线
            &&lineinfo[y+10].right_lost==0
		    &&(get_integeral_state(&distance_integral)==2)/*出环路程积分已完成*/
           ||get_integeral_state(&angle_integral)==2)//出环角度积分已完成
            //&&lineinfo[y].right>10//右边线可以检测到
            //&&lineinfo[y+10].right>10//
            //&&(188-(lineinfo[y].right-lineinfo[y+10].right)*(115-y)/10)>5//右边线斜率达到一定值
            //&&lineinfo[y].right>lineinfo[y+10].right//
            //&&get_integeral_state(&distance_integral)==2/*出环路程积分已完成*/)
             )//下面是原版祖传
            {
				clear_distant_integeral();//清空出环路程积分
				clear_angle_integeral();//清空出环角度积分
//            if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
//            {
//                begin_distant_integeral(setpara.loop_out_distance);
//            }
//            else//如果是大环
//            {
//                begin_distant_integeral(setpara.big_loop_out_distance);
//            }
				begin_distant_integeral(setpara.LeaveLoop_DisIntegral);//开始离开圆环路程积分
				watch.InLoop =5;//出圆环后直行，不用陀螺仪，用摄像头自身提取赛道元素；
            }
    }
}
/*****************************************************************************
***@breif	检测出左环进入直线后左侧角点
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_straight_out_angle()
{
    if(watch.InLoop != 5&&watch.OutLoop!=1)//不是出环后直行的状态
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
//        if((watch.InLoop==5)&&watch.OutLoop==1&&
//            watch.OutLoopAngle2==120&&//看上角点标志位坐标是否已被清空(清空后角点纵坐标变为120)
//			y<90&&y<(watch.watch_lost-10)&&//角点不在屏幕上边
//			(lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30&&
//			(lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30&&
//			(lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30
//			/*角点上方不丢线，下方出现丢线*/)
		 if((watch.InLoop==5)&&watch.OutLoop==1//看当前出入环标志位是否正确
          &&y<(watch.watch_lost-10)//角点不在屏幕上边
          &&y<100//横坐标低于某值(这里改了)
          &&y<watch.OutLoopAngle2
          &&lineinfo[y].left<120//角点所在行左右边线横坐标在中间(之前学长给注释了)
          &&lineinfo[y].right>60	
          &&(lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30
          &&(lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30
&&(lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30//下方丢线，左右坐标值差明显大于上方
          &&lineinfo[y+1].right-lineinfo[y+2].right<5	//上方边线连续,横坐标没有过大偏差
          &&!lineinfo[y].left_lost
          &&!lineinfo[y+1].left_lost
          &&!lineinfo[y+2].left_lost/*角点上方不丢线，下方出现丢线*/)//下面的是原版祖传
          {
			//记录检测到的直行离开时的角点坐标
				watch.OutLoopAngle2 = y;
				watch.OutLoopAngle2_x=lineinfo[ watch.OutLoopAngle2].left;
          }
	}				
	if((watch.InLoop==5)&&watch.OutLoop==1&&watch.OutLoopAngle2!=120)//已经检测到出环直行时的角点坐标
	//这里祖传没有
	{
		find_angle_left_down(watch.OutLoopAngle2_x,watch.OutLoopAngle2,&watch.OutLoopAngle2_x,&watch.OutLoopAngle2);
	}
}
/*****************************************************************************
***@breif	检测完全出环
***@param	无
***@retval	无
*******************************************************************************/
void left_ring_complete_out()
{
    if(watch.InLoop != 5&&watch.OutLoop!=1)
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
		if (watch.InLoop == 5
			&&get_integeral_state(&distance_integral)==2
        	//&&watch.OutLoopAngle2<80
			) //第三个条件祖传有需要调
		{
			clear_all_flags();				//出环成功,清除所有标志
			out_element();					//退出圆环元素
			change_trailing_para(None);		//恢复到正常巡线参数(这里逆透视参数会改)
			LEDRED_OFF;						//关灯
		}
   }
}
/*====================================以下为右环代码===================================================*/
/*****************************************************************************
***@breif	检测右环第一个角点(Inloop=1)
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_first_angle()
{
    if(watch.InLoop != 0&&watch.InLoop!=6)//不是右环的入环阶段
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
//        if (y < 75&&
//          !lineinfo[y - 3].right_lost&&!lineinfo[y - 2].right_lost&&!lineinfo[y - 1].right_lost&&
//		  !lineinfo[y].right_lost&&lineinfo[y + 1].right_lost&&//右环下边不丢线,上面丢线
//         //&&lineinfo[y + 3].right_lost
//         //&&lineinfo[y + 2].right_lost
//         !lineinfo[y +1].left_lost&&!lineinfo[y + 3].left_lost&&!lineinfo[y + 2].left_lost&&
//         !lineinfo[y + 1].left_lost&&!lineinfo[y - 3].left_lost&&!lineinfo[y - 2].left_lost&&
//         !lineinfo[y - 1].left_lost&&!lineinfo[y - 4].left_lost&&//左边不丢线
//         y<watch.InLoopAngleR&&//看标志位坐标是否已被清空(清空后角点纵坐标变为120)
//         lineinfo[y+4].right-lineinfo[y].right>10/*右边线出现跳变*/)
//    //     &&abs_m(lineinfo[y - 5].right,lineinfo[y - 4].right)<8
//    //     &&abs_m(lineinfo[y - 6].right,lineinfo[y - 5].right)<8
		if (
          y < 75
         &&((lineinfo[y + 1].right_lost)//右环下边不丢线,上面丢线
		||(lineinfo[y+1].right-lineinfo[y].right>=5*(lineinfo[y+2].right-lineinfo[y+1].right)))
         && !lineinfo[y - 3].right_lost
         && !lineinfo[y - 2].right_lost
         &&!lineinfo[y - 1].right_lost
         &&!lineinfo[y].right_lost
         &&!lineinfo[y + 5].left_lost//左边不丢线
         &&!lineinfo[y + 3].left_lost
         &&!lineinfo[y + 2].left_lost
         &&!lineinfo[y + 1].left_lost
         &&!lineinfo[y].left_lost
         &&!lineinfo[y - 5].left_lost
         &&!lineinfo[y - 4].left_lost
         &&!lineinfo[y - 3].left_lost
         &&!lineinfo[y - 2].left_lost
         &&!lineinfo[y - 1].left_lost
         &&y<watch.InLoopAngleR//看标志位坐标是否已被清空(清空后角点纵坐标变为120)
         &&lineinfo[y+4].right-lineinfo[y].right>10/*右边线出现跳变*/
         )//下面的是原版祖传
          { 
              watch.InLoopAngleR = y;//左圆环的第一个角点所在行
            //Element=Rifht_ring_confirm;
              if(Element==None) //在当前无元素时进行以下操作，其他时候只找角点
              {
                  right_ring_confirm();       //进入右环二次确认
              }
         }
    }
}
/*****************************************************************************
***@breif	右环二次确认函数
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_confirm()
{
	uint8 white_count1=0;	//第一角点所在行比角点横坐标小的部分的白点总个数
	uint8 white_count2=0;   //第一角点上一行比角点横坐标小的部分的白点总个数
	uint8 white_count3=0;   //第一角点上上一行比角点横坐标小的部分的白点总个数
	uint8 black_count=0;    //第一角点所在列比角点纵坐标小的部分的黑点总个数
	uint8 left_lost=0;      //左侧是否丢线标志位
    //for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    for(int y=loop_forward_near;y<95;y++)//逐行扫描(这行是祖传)
    {
//         if((lineinfo[y].left-lineinfo[y+2].left)>3)//左线临近坐标点出现较大变化
//		 {
//                left_lost=1;//认为左侧出现丢线
//		 }
		 if((lineinfo[y].left-lineinfo[y+2].left>2)//下面的是祖传
			 ||(lineinfo[y+2].left-lineinfo[y].left>4))//左线临近坐标点出现较大变化
		 {
                left_lost+=1;//认为左侧出现丢线
		 }
    }
        if(left_lost<3)//左线连续再进行后续判断
        {
			for(int x=lineinfo[watch.InLoopAngleR-1].right;x<188;x++)//遍历右环第一个角点后边的横坐标
            {//祖传把白点统计注释了
				//计算识别到的右环第一角点附近三行的白点总数
				if(Grayscale[119-watch.InLoopAngleR][x]==255)
				{
					white_count1++;
				}
				if(Grayscale[119-watch.InLoopAngleR-1][x]==255)
				{
					white_count2++;
				}
				if(Grayscale[119-watch.InLoopAngleR-2][x]==255)
				{
					white_count3++;
				}
			}
            for(int y=watch.InLoopAngleR;y>loop_forward_near;y--)//遍历右环第一个角点下边的纵坐标
            {
				//统计右环第一角点所在列的黑点总数
				if(Grayscale[119-y][lineinfo[watch.InLoopAngleR].right]==0)
				{
                     black_count++;
				}
            }
            if((white_count1>=10&&white_count2>=10&&white_count3>=10)&&black_count<10)
				//横行大量白点，纵列少量黑点(祖传没有白点统计)
            {
                    //Element=Right_ring;        
                    enter_element(Right_ring);//正式进入右环元素
					LEDGREEN_ON;									
//                  if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
//                  {
//                      set_speed(setpara.loop_target_speed);
//                      change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
//                  }
//                  else//如果是大环
//                  {
//                      set_speed(setpara.big_loop_speed);
//                      change_pid_para(&CAM_Turn,&setpara.big_loop_PID);
//                  }
                    begin_distant_integeral(setpara.GoInLoop_DisIntegral+1000);//开启路程积分(这里要手动调4750+)
                    watch.InLoop = 6;//检测到右环第一个角点，此时补左侧第一条线保持直行
                    return;//退出检测
                }
        }
        Element=None;		//不满足上述条件，认为不是圆环
        watch.InLoopAngleR=120;//初始化第一角点位置
        watch.InLoopAngleL=120;
        return;
}
/*****************************************************************************
***@breif	检测右环入环第一个角点后的圆弧
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_circular_arc()
{
    if(watch.InLoop != 6&&watch.InLoop != 7)//不是右环的入环阶段
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
//        if (//(watch.InLoop == 6)&&y <watch.InLoopAngle2 &&
//             y>watch.InLoopAngleR&&//圆弧切点在第一角点上方
//			y <watch.InLoopCirc&&//圆弧切点纵坐标值在合理范围内
//			!lineinfo[y+3].right_lost&&!lineinfo[y+2].right_lost&&!lineinfo[y+1].right_lost&&
//			!lineinfo[y-2].right_lost&&!lineinfo[y-1].right_lost&&!lineinfo[y-3].right_lost&&
//			//左侧不丢线
//			lineinfo[y+1].right >= lineinfo[y].right&&lineinfo[y+2].right >= lineinfo[y+1].right&&
//			lineinfo[y+3].right <= lineinfo[y].right&&lineinfo[y-1].right >= lineinfo[y].right&&
//			lineinfo[y-2].right >= lineinfo[y-1].right&&lineinfo[y-3].right <= lineinfo[y].right
//			/*右侧某点横坐标大于周围点，认为是圆环的圆弧切点*/
//           )
		 if (//(watch.InLoop == 6)&&y <watch.InLoopAngle2 &&
         watch.InLoopAngleR<55//入环角点坐标处于一定范围内
         &&(y>(watch.InLoopAngleR+20)
		 &&get_integeral_state(&distance_integral)==2)//前一段路程积分完成
           &&y <watch.InLoopCirc//圆弧切点在第一角点上方
           &&!lineinfo[y+3].right_lost//右侧不丢线(到了圆环中间的圆形区域)
           &&!lineinfo[y+2].right_lost
           &&!lineinfo[y+1].right_lost
           &&!lineinfo[y-3].right_lost
           &&!lineinfo[y-2].right_lost
           &&!lineinfo[y-1].right_lost
           &&lineinfo[y+1].right >= lineinfo[y].right/*右侧某点横坐标大于周围点，认为是圆环的圆弧切点*/
           &&lineinfo[y+2].right >= lineinfo[y+1].right
           &&lineinfo[y+3].right >= lineinfo[y].right
           &&lineinfo[y-1].right >= lineinfo[y].right
           &&lineinfo[y-2].right >= lineinfo[y-1].right
           &&lineinfo[y-3].right >= lineinfo[y].right
           )//下面是原版祖传
           { 
                watch.InLoopCirc = y;//圆环上凸弧(切点)所在行
                break;
           }
    }
}
/*****************************************************************************
***@breif	检测右环第二个角点(入口角点)
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_second_angle()
{
    if(watch.InLoop != 6&&watch.InLoop != 7)//不是右环的入环阶段
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描最靠下的点，近距离时较为靠谱（第一判据）
    {
//        if(watch.InLoopCirc<120&&//切点处还未被识别到
//           y<watch.InLoopAngle2&&//当前点处于正常范围内
//           watch.InLoopAngle2==120&&//之前第二角点坐标已被清空
//           get_integeral_state(&distance_integral)==2&&//上一段路程积分已完成
//           60<y&&y < (loop_forward_far-2)&&//当前行在图像上半部分
//           y>watch.InLoopCirc&&//当前行在圆环切点上方
//           //&&lineinfo[watch.InLoopCirc].right_lost
//           lineinfo[y+1].right <158&&
//           (lineinfo[y].right-lineinfo[y+1].right)<=2&&//当前行上边连续
//           (lineinfo[y-4].right-lineinfo[y].right)>lineinfo[y].right/3/*当前行下方点出现跳变(丢线)*/)
		if(watch.InLoopCirc<66//切点到屏幕下面
               &&y<watch.InLoopAngle2//当前点处于正常范围内
               &&watch.InLoopAngle2==120//之前第二角点坐标已被清空
               &&get_integeral_state(&distance_integral)==2//上一段路程积分已完成
               &&y > 60//当前行在图像上半部分
               &&y < (loop_forward_far-2)
               &&y>watch.InLoopCirc//当前行在圆环切点上方
               &&lineinfo[y+1].right <158
               &&(lineinfo[y].right-lineinfo[y+1].right)<=2//当前行上边连续
             &&(lineinfo[y-4].right-lineinfo[y].right)>(187-lineinfo[y].right)/2/*当前行下方点出现跳变(丢线)*/
          )//上面那行动过了
           {
			   //记录右环第二角点坐标
				watch.InLoopAngle2 = y;
				watch.InLoopAngle2_x=lineinfo[watch.InLoopAngle2].right;
				break;//找到后退出
           }
    }
    if(watch.InLoopAngle2!=120&&watch.InLoopAngle2>50)//左环第二角点纵坐标在图像上边
    {
        find_angle_right_down(watch.InLoopAngle2_x,watch.InLoopAngle2,&watch.InLoopAngle2_x,
		&watch.InLoopAngle2);//向右下找角点不太清除这句干嘛用的
    }
}
/*****************************************************************************
***@breif	右环开始转向函数
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_begin_turn()
{
    if(watch.InLoop != 6)//不是入环状态
	{
		return;//在循环之前跳出，节省时间
	}
    if(get_integeral_state(&distance_integral)==2&&//路程积分完成
       watch.InLoop==6&&
       (watch.InLoopAngle2<=75)/*右环第二角点现在离车较近*/)//这里动过了
    {
        clear_distant_integeral();//清除路程积分变量(准备环内积分) 
		watch.InLoop=7;//更改标志位，正在入环，此时左侧补线入环
		begin_angle_integeral(-setpara.InLoop_AngleIntegral);//开始转向角度积分(注意是负值)
      //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
      //set_speed(setpara.loop_target_speed);
      //watch.fix_slope=(float)(188-lineinfo[watch.InLoopAngle2].right)/(115-watch.InLoopAngle2);
    }
}
/*****************************************************************************
***@breif	检验小车是否完全入右环
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_in_loop()
{
    if(watch.InLoop != 7)//现在不是拉线入环阶段
	{
		return;//在循环之前跳出，节省时间
	}
	
    if( watch.InLoop == 7//正在进行圆环内部路程积分
        &&get_integeral_state(&angle_integral)==1
        &&get_integeral_data(&angle_integral)<-40/*转过一定角度*/)
        //&&watch.left_near_lost<40
    {
		watch.InLoop = 8;//更改标志位，此时完全入环
		change_trailing_para(Right_ring);//更改循迹参数为右环的 (后期提速用)    
      //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
    }
}
/*****************************************************************************
***@breif	小车角度积分完成，准备出环
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_prepare_out()
{
    if(watch.InLoop != 8)//小车不是入环状态
	{
		return;//在循环之前跳出，节省时间
	}
    if( watch.InLoop == 8&&
        get_integeral_state(&angle_integral)==1&&//正在进行环内角度积分
        get_integeral_data(&angle_integral)<-180/*积分已经达到一定值*/
		&&lineinfo[69].left>35
        &&lineinfo[69].left<105)//后两句是祖传原版的
   {
       watch.InLoop = 9;//更改标志位，陀螺仪积分完成，准备出环
       watch.OutLoop_turn_point_x=lineinfo[69].left;//转向点横坐标，根据该点进行补线(为何这么取)
   }
}
/*****************************************************************************
***@breif	检测出环时左角点位置
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_out_angle()
{
    if(watch.InLoop != 9)//当前不是准备出环状态
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
        if ((watch.InLoop == 9) &&y<80&&
             //lineinfo[y].left_lost
             lineinfo[y + 1].left <= lineinfo[y].left&&
             lineinfo[y+2].left <= lineinfo[y+1].left&&
             lineinfo[y-1].left <= lineinfo[y].left&&
             lineinfo[y-2].left <= lineinfo[y].left&&//右圆环出环角点的横坐标值大于附近的点
             lineinfo[y].left <158&&y>30&&//角点坐标在图像左边
             Grayscale[119-y-2][lineinfo[y].left]==255/*角点上方点为白点*/)
             {
                 if(watch.OutLoopAngle1>y)//之前未检测到角点(清空后角点纵坐标变为120)
                 {
                 //watch.OutLoopLeft = lineinfo[y].left;
                     watch.OutLoopAngle1 = y; //记录出环角点纵坐标
                 }
             }
   }
}
/*****************************************************************************
***@breif	出右环右转
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_out_loop_turn()
{
    if(watch.InLoop == 9
       //&&(watch.cross+watch.left_lost)>30//右侧丢线过多，说明进入圆环与直线交界处
       &&watch.OutLoopAngle1<120
       &&get_integeral_state(&angle_integral)==2//环内路程积分已经完成
       &&lineinfo[watch.OutLoopAngle1].left_lost==1/*左侧丢线(出环时左边是横着的)*/
       &&watch.OutLoop==0)
    {
        clear_angle_integeral();//清空积分标志位和数值
		begin_angle_integeral(setpara.OutLoop_AngleIntegral);//开启出环转向积分
        begin_distant_integeral(setpara.OutLoop_DisIntegral);//开启路程积分，此时要保持右转(这里要调)
		//change_trailing_para(None);   //回到正常巡线的参数
        watch.OutLoop=1;//出环标志为置1已经出环
    }
}
/*****************************************************************************
***@breif	左侧为直线时直行
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_out_loop()
{
    if(watch.InLoop != 9&&watch.OutLoop!=1)//当前不是出环或出环后状态
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
//        if((watch.InLoop == 9&&watch.OutLoop==1&&//看当前出入环标志位是否正确
//            y<60&&
//            lineinfo[y].left_lost==0&&lineinfo[y+10].left_lost==0&&//左边不再丢线
//            ((lineinfo[y+10].left-lineinfo[y].left)*(115-y)/10)<148&&//？？？？？
//            get_integeral_state(&distance_integral)==2)/*出环路程积分已完成*/)
			if((watch.InLoop == 9
				&&watch.OutLoop==1//看当前出入环标志位是否正确
				&&y<60
				&&lineinfo[y].left_lost==0//左边不再丢线
				&&lineinfo[y+10].left_lost==0
				&&lineinfo[y].left>10//////////左边线不在屏幕边缘(这两句是祖传加的)
				&&lineinfo[y+10].left>10///////
				&&((lineinfo[y+10].left-lineinfo[y].left)*(115-y)/10)<148//左边线斜率达到一定值
				&&lineinfo[y+10].left-lineinfo[y].left>0//左边线上点坐标有一定差值(这句是祖传的)
				&&get_integeral_state(&distance_integral)==2/*出环路程积分已完成*/)
                ||(get_integeral_state(&angle_integral)==2/*出环转向角度积分已完成*/)
				)
                
         {
            clear_distant_integeral();//清空出环路程积分
			clear_angle_integeral();//清空出环转向角度积分
//          if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
//          {
//              begin_distant_integeral(setpara.loop_out_distance);
//          }
//          else//如果是大环
//          {
//              begin_distant_integeral(setpara.big_loop_out_distance);
//          }
			begin_distant_integeral(setpara.LeaveLoop_DisIntegral);//开始离开圆环路程积分(这里要调)
			//change_trailing_para(None);   //回到正常巡线的参数
			watch.InLoop =10;//出圆环后直行，不用陀螺仪，用摄像头自身提取赛道元素；
			watch.OutLoop=1;//将要出环
         }
     }
}
/*****************************************************************************
***@breif	检测出右环进入直线后右侧角点
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_straight_out_angle()
{
	if(watch.InLoop != 10&&watch.OutLoop!=1)//不是出环后直行的状态
	{
		return;//在循环之前跳出，节省时间
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
//        if((watch.InLoop==10)&&watch.OutLoop==1&&//看当前出入环标志位是否正确
//          y<watch.OutLoopAngle2&&//看上角点标志位坐标是否已被清空(清空后角点纵坐标变为120)
//          y<80&&y<(watch.watch_lost-10) &&lineinfo[y].left<120&&lineinfo[y].right>60&&//角点不在屏幕上边
//          watch.zebra_flag == 0&&//当前无斑马线
//          (lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30&&
//          (lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30&&
//          (lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30&&
//          lineinfo[y+2].left-lineinfo[y+1].left<5/*角点上方不丢线，下方出现丢线*/)
		if((watch.InLoop==10)&&watch.OutLoop==1//看当前出入环标志位是否正确
          &&y<(watch.watch_lost-10)//角点不在屏幕上边
          &&y<80
          &&y<watch.OutLoopAngle2//看上角点标志位坐标是否已被清空(清空后角点纵坐标变为120)
          &&lineinfo[y].left<120//角点所在行左右边界点在中间
          &&lineinfo[y].right>60
          //&& watch.zebra_flag == 0
          &&(lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30
          &&(lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30
          &&(lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30
          &&lineinfo[y+2].left-lineinfo[y+1].left<5/*角点上方不丢线，下方出现丢线*/
          &&!lineinfo[y].right_lost//右侧出现丢线
          &&!lineinfo[y+1].right_lost
          &&!lineinfo[y+2].right_lost)//下面的是原版祖传
		 {
			 //记录检测到的直行离开时的第一角点坐标
             watch.OutLoopAngle2 = y;
		     watch.OutLoopAngle2_x = lineinfo[watch.OutLoopAngle2].right;
         }
    }
}
/*****************************************************************************
***@breif	右环检测完全出环
***@param	无
***@retval	无
*******************************************************************************/
void right_ring_complete_out()
{
    if (watch.InLoop == 10
		&&get_integeral_state(&distance_integral)==2
		//&&watch.OutLoopAngle2<25
		)  
     {  
         clear_all_flags();                 //出环成功,清除所有标志
         out_element();                     //退出圆环元素
		 LEDGREEN_OFF;                      //关灯
//		 element_number ++;                 //19届遗留状态机
//		 element_timer_temp = timer.timer_ms;
     //mycar.target_speed=setpara.com_target_speed;//恢复正常速度
		change_trailing_para(None);   //回到正常巡线的参数
	 }
}
//向左下找角点
void find_angle_left_down(uint8 x,uint8 y,int*angle_x,int*angle_y)
{
    while(Grayscale[119-y][x]!=0&&y<110)
    {
        y++;
    }
    while(Grayscale[119-y][x+1]!=255&&x<187)
    {
        x++;
    }
    while(y>40)
    {
        if(Grayscale[119-y][x]==0)
        {

        }
        else if(Grayscale[119-y][x-1]==0)
        {
            x--;
        }
        else if(Grayscale[119-y][x+1]==0)
        {
            x++;
        }
        else if(Grayscale[119-y][x-2]==0)
        {
            x=x-2;
        }
        else if(Grayscale[119-y][x+2]==0)
        {
            x=x+2;
        }
        else if(Grayscale[119-y][x-3]==0)
        {
            x=x-3;
        }
        else if(Grayscale[119-y][x+3]==0)
        {
            x=x+3;
        }
        else break;
        y--;
    }
    *angle_x=x;
    *angle_y=y;
}
//向右下找角点
void find_angle_right_down(uint8 x,uint8 y,int*angle_x,int*angle_y)
{
    while(Grayscale[119-y][x]!=0&&y<110)
    {
        y++;
    }
    while(Grayscale[119-y][x-1]!=255&&x>0)
    {
        x--;
    }
    while(y>40)
    {
        if(Grayscale[119-y][x]==0)
        {

        }
        else if(Grayscale[119-y][x-1]==0)
        {
            x--;
        }
        else if(Grayscale[119-y][x+1]==0)
        {
            x++;
        }
        else if(Grayscale[119-y][x-2]==0)
        {
            x=x-2;
        }
        else if(Grayscale[119-y][x+2]==0)
        {
            x=x+2;
        }
        else if(Grayscale[119-y][x-3]==0)
        {
            x=x-3;
        }
        else if(Grayscale[119-y][x+3]==0)
        {
            x=x+3;
        }
        else break;
        y--;
    }
    *angle_x=x;
    *angle_y=y;
}
