/*˵��:���ļ�����������Բ���ĸ�����־λ���жϺ���(��һ�ǵ㣬�е㣬�ڶ��ǵ㣬�����ǵ㣬����ֱ�нǵ�)��û��
���ߵĲ���
 * circle.c
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */
#include "zf_common_headfile.h"

#define loop_forward_far  100
#define loop_forward_near 25

//extern int element_number;
//extern int16 element_timer_temp;

/*****************************************************************************
***@breif	�뻷ʱ����󻷵�һ���ǵ�(Inloop=1)
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_first_angle()
{
    if(watch.InLoop!=0&&watch.InLoop!=1)//�����󻷵��뻷�׶�
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
//        if(//lineinfo[y + 3].left_lost
//            //&&lineinfo[y + 2].left_lost&&
//            lineinfo[y + 1].left_lost&&  //�����Ϸ����ֶ���
//			!lineinfo[y - 3].left_lost&&!lineinfo[y - 2].left_lost&&!lineinfo[y - 4].left_lost&&\
//            !lineinfo[y].left_lost&&    //�����·�����������
//            !lineinfo[y+1].right_lost&&!lineinfo[y].right_lost&&!lineinfo[y-1].right_lost&&//������������
//			lineinfo[y].left-lineinfo[y+4].left>10//���ߺ������������
//            &&y<watch.InLoopAngleL&&y<75)//���������д��ں���Χ��
            //&&lineinfo[y].left>=lineinfo[y-2].left
		if(((lineinfo[y + 1].left_lost)||  //�����Ϸ����ֶ���
			((lineinfo[y].left-lineinfo[y+1].left)>=5*(lineinfo[y+1].left-lineinfo[y+2].left)))
            && !lineinfo[y - 3].left_lost//�����·�����������
            && !lineinfo[y - 2].left_lost
            && !lineinfo[y - 1].left_lost
            && !lineinfo[y].left_lost
            && !lineinfo[y + 5].right_lost//������������
            && !lineinfo[y + 3].right_lost
            && !lineinfo[y + 2].right_lost
            && !lineinfo[y + 1].right_lost
            && !lineinfo[y].right_lost
            && !lineinfo[y - 1].right_lost
            && !lineinfo[y - 2].right_lost
            && !lineinfo[y - 3].right_lost
            && !lineinfo[y - 4].right_lost
            && !lineinfo[y - 5].right_lost
            &&lineinfo[y].left-lineinfo[y+4].left>10 //���ߺ������������
            &&y<watch.InLoopAngleL
            //&&lineinfo[y].left>=lineinfo[y-2].left
            &&y<75)  //���������д��ں���Χ��
        {
            watch.InLoopAngleL = y;//��¼����ǰֱ�еĵ�һ����������
            if(Element==None) //�ڵ�ǰ��Ԫ��ʱ�������²���������ʱ��ֻ�ҽǵ�
            {
				left_ring_confirm();//�����󻷶���ȷ��
            }
            break;//ʶ��֮���˳�ѭ��
        }
    }
}
/*****************************************************************************
***@breif	���뻷����ȷ�Ϻ���
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_confirm()
{
	uint8 white_count1=0;//��һ�ǵ������бȽǵ������С�Ĳ��ֵİ׵��ܸ���
	uint8 white_count2=0;//��һ�ǵ���һ�бȽǵ������С�Ĳ��ֵİ׵��ܸ���
	uint8 white_count3=0;//��һ�ǵ�����һ�бȽǵ������С�Ĳ��ֵİ׵��ܸ���
	uint8 black_count=0; //��һ�ǵ������бȽǵ�������С�Ĳ��ֵĺڵ��ܸ���
	uint8 right_lost=0;	 //�Ҳ��Ƿ��߱�־λ
    //for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
	for(int y=loop_forward_near;y<95;y++)//����ɨ�裨��ѧ���ģ�
    {
//        if((lineinfo[y+2].right-lineinfo[y].right)>3)//�����ٽ��������ֽϴ�仯
//		{
//            right_lost=1;//��Ϊ�Ҳ���ֶ���
//		}
		if(((lineinfo[y+2].right-lineinfo[y].right)>2)||
			(lineinfo[y].right-lineinfo[y+2].right)>4)//�����ٽ��������ֽϴ�仯(�洫ԭ��)
		{
			right_lost+=1;//��Ϊ�Ҳ���ֶ���
		}
    }
    if(right_lost<3)//��Ϊ���������ٽ��к����ж�
    {
		for(int x=lineinfo[watch.InLoopAngleL-1].left;x>0;x--)//�����󻷵�һ���ǵ��ߵĺ�����
		{
			//����ʶ�𵽵��󻷵�һ�ǵ㸽�����еİ׵�����
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
        for(int y=watch.InLoopAngleL;y>loop_forward_near;y--)//�����󻷵�һ���ǵ��±ߵ�������
        {
            if(Grayscale[119-y][lineinfo[watch.InLoopAngleL].left]==0)//ͳ���󻷵�һ�ǵ������еĺڵ�����
			{
               black_count++;
			}
        }			
        if((white_count1>=10&&white_count2>=10&&white_count3>=10)&&(black_count<10))//�洫ֻ�к�ɫͳ��
		//���д����׵㣬���������ڵ�
        {			
            enter_element(Left_ring);    //��ʽ������Բ��Ԫ��
            begin_distant_integeral(setpara.GoInLoop_DisIntegral);//����·�̻���(����Ҫ��)
			LEDRED_ON;	//����
//			set_speed(setpara.loop_target_speed);
//          change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//��ת��PID������Ϊ����ת��PID(����������˵)
			watch.InLoop = 1;	 //��⵽�󻷵�һ���ǵ㣬��ʱ������һ���߱���ֱ��
			return;	 			 //�˳����
/*          else//����Ǵ�
            {
                  set_speed(setpara.big_loop_speed);
                  //change_pid_para(&CAM_Turn,&setpara.big_loop_PID);
            }*/
         }
    }
	Element=None;//������������������Ϊ����Բ��
    watch.InLoopAngleR=120;//��ʼ����һ�ǵ�λ��
    watch.InLoopAngleL=120;
}
/*****************************************************************************
***@breif	������뻷��һ���ǵ���Բ��
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_circular_arc()
{
    if (watch.InLoop != 1&&watch.InLoop != 2)//�����󻷵��뻷�׶�
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
        if (/*//y <watch.InLoopAngle2  &&
             y>watch.InLoopAngleL//Բ���е��ڵ�һ�ǵ��Ϸ�
			 &&y<watch.InLoopCirc//Բ���е�������ֵ�ں���Χ��
           &&!lineinfo[y+3].left_lost&&!lineinfo[y+2].left_lost
		   &&!lineinfo[y+1].left_lost&&!lineinfo[y-3].left_lost
           &&!lineinfo[y-2].left_lost&&!lineinfo[y-1].left_lost//��಻����
           &&lineinfo[y+1].left <= lineinfo[y].left&&lineinfo[y+2].left <= lineinfo[y].left
           &&lineinfo[y+3].left <= lineinfo[y].left&&lineinfo[y-1].left <= lineinfo[y].left
		   &&lineinfo[y-2].left <= lineinfo[y].left&&lineinfo[y-3].left <= lineinfo[y].left
           //���ĳ������������Χ�㣬��Ϊ��Բ����Բ���е�*/       
			(watch.InLoopAngleL<65||get_integeral_state(&distance_integral)==2)//Բ���е��ڵ�һ�ǵ��Ϸ�
           &&(y>(watch.InLoopAngleL+20)||get_integeral_state(&distance_integral)==2)//·�̻��ֽ���ֱ�ӽ���һ״̬
           &&y <watch.InLoopCirc//Բ���е�������ֵ�ں���Χ��
           &&!lineinfo[y+3].left_lost
           &&!lineinfo[y+2].left_lost
           &&!lineinfo[y+1].left_lost
           &&!lineinfo[y-3].left_lost
           &&!lineinfo[y-2].left_lost
           &&!lineinfo[y-1].left_lost//��಻����
           &&lineinfo[y+1].left <= lineinfo[y].left
           &&lineinfo[y+2].left <= lineinfo[y].left
           &&lineinfo[y+3].left <= lineinfo[y].left
           &&lineinfo[y-1].left <= lineinfo[y].left
           &&lineinfo[y-2].left <= lineinfo[y].left
           &&lineinfo[y-3].left <= lineinfo[y].left//���ĳ������������Χ�㣬��Ϊ��Բ����Բ���е�
		)//�������ԭ���洫
       { 
            watch.InLoopCirc = y;//Բ����͹��(�е�)������
            break;
       }
    }
}
/*****************************************************************************
***@breif	����󻷵ڶ����ǵ�(��ڽǵ�)
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_second_angle()
{
    if(watch.InLoop != 1&&watch.InLoop != 2)//�����󻷵��뻷�׶�
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
//        if (watch.InLoopCirc<120&&//�е㴦��δ��ʶ��
//            y<watch.InLoopAngle2&&//��ǰ�㴦��������Χ��
//            watch.InLoopAngle2==120&&//֮ǰ�ڶ��ǵ������ѱ����
//            get_integeral_state(&distance_integral)==2&&//��һ��·�̻��������
//			y < 60&&y <(loop_forward_far-2)&& //��ǰ����ͼ���ϰ벿��			
//			y>watch.InLoopCirc&&//��ǰ����Բ���е��Ϸ�
//           lineinfo[y+1].left > 30&&(lineinfo[y+1].left-lineinfo[y].left)<=2&&//��ǰ���ϱ�����
//           (lineinfo[y].left-lineinfo[y-4].left)>lineinfo[y].left/3/*��ǰ���·����������(����)*/)  
		  if (watch.InLoopCirc<66	
			  &&y<watch.InLoopAngle2//��ǰ�㴦��������Χ��
              &&watch.InLoopAngle2==120//֮ǰ�ڶ��ǵ������ѱ����
              &&get_integeral_state(&distance_integral)==2 //��һ��·�̻��������
			  &&y > 60	//��ǰ����ͼ���ϰ벿��
              &&y < (loop_forward_far-2)
			  &&y>watch.InLoopCirc//��ǰ����Բ���е��Ϸ�
			  &&lineinfo[y+1].left > 30
              &&(lineinfo[y+1].left-lineinfo[y].left)<=2 //��ǰ���ϱ�����
           &&(lineinfo[y].left-lineinfo[y-4].left)>lineinfo[y].left/2/*��ǰ���·����������(����)*/
           )//������ԭ���洫		
           {
			   //��¼�󻷵ڶ��ǵ�����
               watch.InLoopAngle2 = y;
               watch.InLoopAngle2_x=lineinfo[watch.InLoopAngle2].left;
               break;//�ҵ����˳�
           }
    }
    if(watch.InLoopAngle2!=120&&watch.InLoopAngle2>50)//�󻷵ڶ��ǵ���������ͼ���ϱ�
    {
        find_angle_left_down(watch.InLoopAngle2_x,watch.InLoopAngle2,&watch.InLoopAngle2_x,
		&watch.InLoopAngle2);//�������ҽǵ㲻̫����������õ�
    }
}
/*****************************************************************************
***@breif	�󻷿�ʼת����(watch.InLoop=2)
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_begin_turn()
{
	if(watch.InLoop!=1)//�����뻷״̬
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    if(get_integeral_state(&distance_integral)==2//·�̻������
        &&watch.InLoop==1	//��ǰ����δ�뻷�׶�
        //&&lineinfo[watch.InLoopCirc].left_lost
        &&watch.InLoopAngle2<=80/*�󻷵ڶ��ǵ������복�Ͻ�*/)
    {
		watch.InLoop=2;//���ı�־λ�������뻷����ʱ�Ҳಹ���뻷
        clear_distant_integeral();		//���·�̻��ֱ���(׼�����ڻ���)
		change_trailing_para(Left_ring);//��Ѳ�߲�����Ϊ����Ѳ��PID���������ٿ����ã�
        //watch.fix_slope=(float)(lineinfo[watch.InLoopAngle2].left)/(115-watch.InLoopAngle2);
        begin_angle_integeral(setpara.InLoop_AngleIntegral);//��ʼԲ���뻷�ǶȻ���
    }
}
/*****************************************************************************
***@breif	����С���Ƿ���ȫ�뻷
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_in_loop()
{
    if(watch.InLoop != 2)//���ڲ��������뻷�׶�
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    if( watch.InLoop == 2&&
        get_integeral_state(&angle_integral)==1&&//���ڽ���Բ���ڲ��ǶȻ���
        get_integeral_data(&angle_integral)>40/*ת��һ���Ƕ�*/)
        //&&watch.right_near_lost<40
        //&&lineinfo[watch.right_near_lost].right>180
        //&&(!lineinfo[40].right_lost)        
        {
			watch.InLoop = 3;//���ı�־λ����ʱ��ȫ�뻷
            //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//��ת��PID������Ϊ����ת��PID
        }
		
}
/*****************************************************************************
***@breif	С���ǶȻ��ֿ�Ҫ��ɣ�׼������
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_prepare_out()
{
    if(watch.InLoop != 3)//С�������뻷״̬
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    if(watch.InLoop == 3&&
       get_integeral_state(&angle_integral)==1&&//���ڽ��л��ڽǶȻ���
       get_integeral_data(&angle_integral)>180 /*�����Ѿ��ﵽһ��ֵ*/
	   &&lineinfo[69].right<152
       &&lineinfo[69].right>82)//����������ԭ���洫
   {
       watch.InLoop = 4;//���ı�־λ�������ǻ�����ɣ�׼������
       watch.OutLoop_turn_point_x=lineinfo[69].right;//ת�������꣬���ݸõ���в���(Ϊ����ôȡ��)
   }
}
/*****************************************************************************
***@breif	������ʱ�ҽǵ�λ��
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_out_angle()
{
    if(watch.InLoop != 4)//��ǰ����׼������״̬
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
		if ((watch.InLoop == 4)&&y<80
                 //lineinfo[y].left_lost
             &&lineinfo[y+1].right >= lineinfo[y].right
             &&lineinfo[y+2].right >= lineinfo[y+1].right
             &&lineinfo[y-1].right >= lineinfo[y].right
             &&lineinfo[y-2].right >= lineinfo[y].right//��Բ�������ǵ�ĺ�����ֵС�ڸ����ĵ�
/*                 &&lineinfo[y - 3].right > lineinfo[y - 1].right
                 &&lineinfo[y + 4].right > lineinfo[y + 2].right
                 &&lineinfo[y - 5].right > lineinfo[y - 3].right*/
             &&lineinfo[y].right > 30//�ǵ�������ͼ���ϱ�
             &&Grayscale[119-y-2][lineinfo[y].right]==255/*�ǵ��Ϸ���Ϊ�׵�*/)

            {
                 if(watch.OutLoopAngle1>y)//֮ǰΪ��⵽�ǵ�(��պ�ǵ��������Ϊ120)
                 {
                     //watch.OutLoopRight = lineinfo[y].right;
                     watch.OutLoopAngle1 = y; //��¼�����ǵ�������
                     break;
                 }
            }
    }
}
/*****************************************************************************
***@breif	������ת(����Բ����ֱ�߽��紦)
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_out_loop_turn()
{
    if(watch.InLoop != 4)//��ǰ���ǳ���״̬
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    if(watch.InLoop == 4&&
       //&&(watch.cross+watch.right_lost)>30//�Ҳඪ�߹��࣬˵������Բ����ֱ�߽��紦
       watch.OutLoopAngle1<120&&	//��⵽�����Ľǵ�
       get_integeral_state(&angle_integral)==2//���ڽǶȻ����Ѿ����
       //&&lineinfo[watch.OutLoopAngle1].right_lost==1/*�Ҳඪ��(����ʱ�ұ��Ǻ��ŵ�)*/
		&&watch.OutLoop==0)//��δ����(����ӵ�)
    {
        clear_angle_integeral();//��ջ��ֱ�־λ����ֵ
//        if(Element_rem.loop_data[Element_rem.loop_count]==0)//�����С��
//        {
//            begin_angle_integeral(setpara.loop_angle_out);
//        }
//        else//����Ǵ�
//        {
//            begin_angle_integeral(setpara.big_loop_out);
//        }				
		begin_angle_integeral(setpara.OutLoop_AngleIntegral);//��ʼ�����ǶȻ���
        begin_distant_integeral(setpara.OutLoop_DisIntegral);//����·�̻��֣���ʱҪ������ת(������Ҫ��)
        watch.OutLoop=1;//������־Ϊ��1�Ѿ�����
    }
}
/*****************************************************************************
***@breif	�Ҳ�Ϊֱ��ʱֱ��
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_out_loop()
{
    if(watch.InLoop != 4&&watch.OutLoop!=1)//��ǰ���ǳ����������״̬
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
//        if((watch.InLoop == 4&&watch.OutLoop==1&&
//            y<60&&
//            lineinfo[y].right_lost==0&&lineinfo[y+10].right_lost==0&&
//            (188-(lineinfo[y].right-lineinfo[y+10].right)*(115-y)/10)>40&&//����������
//            //&&(abs(lineinfo[watch.right_near_lost].right-lineinfo[watch.right_near_lost+5].right))<20//�Ҳ�Ϊֱ��
//            //&&(watch.cross+watch.right_lost)<15//�Ҳඪ��������
//           get_integeral_state(&distance_integral)==2)/*����·�̻��������*/
//           ||get_integeral_state(&angle_integral)==2)
		if((watch.InLoop == 4  //����ǰ���뻷��־λ�Ƿ���ȷ
            &&watch.OutLoop==1
            &&y<100
            &&lineinfo[y].right_lost==0//�ұ߲��ٶ���
            &&lineinfo[y+10].right_lost==0
		    &&(get_integeral_state(&distance_integral)==2)/*����·�̻��������*/
           ||get_integeral_state(&angle_integral)==2)//�����ǶȻ��������
            //&&lineinfo[y].right>10//�ұ��߿��Լ�⵽
            //&&lineinfo[y+10].right>10//
            //&&(188-(lineinfo[y].right-lineinfo[y+10].right)*(115-y)/10)>5//�ұ���б�ʴﵽһ��ֵ
            //&&lineinfo[y].right>lineinfo[y+10].right//
            //&&get_integeral_state(&distance_integral)==2/*����·�̻��������*/)
             )//������ԭ���洫
            {
				clear_distant_integeral();//��ճ���·�̻���
				clear_angle_integeral();//��ճ����ǶȻ���
//            if(Element_rem.loop_data[Element_rem.loop_count]==0)//�����С��
//            {
//                begin_distant_integeral(setpara.loop_out_distance);
//            }
//            else//����Ǵ�
//            {
//                begin_distant_integeral(setpara.big_loop_out_distance);
//            }
				begin_distant_integeral(setpara.LeaveLoop_DisIntegral);//��ʼ�뿪Բ��·�̻���
				watch.InLoop =5;//��Բ����ֱ�У����������ǣ�������ͷ������ȡ����Ԫ�أ�
            }
    }
}
/*****************************************************************************
***@breif	�����󻷽���ֱ�ߺ����ǵ�
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_straight_out_angle()
{
    if(watch.InLoop != 5&&watch.OutLoop!=1)//���ǳ�����ֱ�е�״̬
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
//        if((watch.InLoop==5)&&watch.OutLoop==1&&
//            watch.OutLoopAngle2==120&&//���Ͻǵ��־λ�����Ƿ��ѱ����(��պ�ǵ��������Ϊ120)
//			y<90&&y<(watch.watch_lost-10)&&//�ǵ㲻����Ļ�ϱ�
//			(lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30&&
//			(lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30&&
//			(lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30
//			/*�ǵ��Ϸ������ߣ��·����ֶ���*/)
		 if((watch.InLoop==5)&&watch.OutLoop==1//����ǰ���뻷��־λ�Ƿ���ȷ
          &&y<(watch.watch_lost-10)//�ǵ㲻����Ļ�ϱ�
          &&y<100//���������ĳֵ(�������)
          &&y<watch.OutLoopAngle2
          &&lineinfo[y].left<120//�ǵ����������ұ��ߺ��������м�(֮ǰѧ����ע����)
          &&lineinfo[y].right>60	
          &&(lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30
          &&(lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30
&&(lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30//�·����ߣ���������ֵ�����Դ����Ϸ�
          &&lineinfo[y+1].right-lineinfo[y+2].right<5	//�Ϸ���������,������û�й���ƫ��
          &&!lineinfo[y].left_lost
          &&!lineinfo[y+1].left_lost
          &&!lineinfo[y+2].left_lost/*�ǵ��Ϸ������ߣ��·����ֶ���*/)//�������ԭ���洫
          {
			//��¼��⵽��ֱ���뿪ʱ�Ľǵ�����
				watch.OutLoopAngle2 = y;
				watch.OutLoopAngle2_x=lineinfo[ watch.OutLoopAngle2].left;
          }
	}				
	if((watch.InLoop==5)&&watch.OutLoop==1&&watch.OutLoopAngle2!=120)//�Ѿ���⵽����ֱ��ʱ�Ľǵ�����
	//�����洫û��
	{
		find_angle_left_down(watch.OutLoopAngle2_x,watch.OutLoopAngle2,&watch.OutLoopAngle2_x,&watch.OutLoopAngle2);
	}
}
/*****************************************************************************
***@breif	�����ȫ����
***@param	��
***@retval	��
*******************************************************************************/
void left_ring_complete_out()
{
    if(watch.InLoop != 5&&watch.OutLoop!=1)
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
		if (watch.InLoop == 5
			&&get_integeral_state(&distance_integral)==2
        	//&&watch.OutLoopAngle2<80
			) //�����������洫����Ҫ��
		{
			clear_all_flags();				//�����ɹ�,������б�־
			out_element();					//�˳�Բ��Ԫ��
			change_trailing_para(None);		//�ָ�������Ѳ�߲���(������͸�Ӳ������)
			LEDRED_OFF;						//�ص�
		}
   }
}
/*====================================����Ϊ�һ�����===================================================*/
/*****************************************************************************
***@breif	����һ���һ���ǵ�(Inloop=1)
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_first_angle()
{
    if(watch.InLoop != 0&&watch.InLoop!=6)//�����һ����뻷�׶�
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
//        if (y < 75&&
//          !lineinfo[y - 3].right_lost&&!lineinfo[y - 2].right_lost&&!lineinfo[y - 1].right_lost&&
//		  !lineinfo[y].right_lost&&lineinfo[y + 1].right_lost&&//�һ��±߲�����,���涪��
//         //&&lineinfo[y + 3].right_lost
//         //&&lineinfo[y + 2].right_lost
//         !lineinfo[y +1].left_lost&&!lineinfo[y + 3].left_lost&&!lineinfo[y + 2].left_lost&&
//         !lineinfo[y + 1].left_lost&&!lineinfo[y - 3].left_lost&&!lineinfo[y - 2].left_lost&&
//         !lineinfo[y - 1].left_lost&&!lineinfo[y - 4].left_lost&&//��߲�����
//         y<watch.InLoopAngleR&&//����־λ�����Ƿ��ѱ����(��պ�ǵ��������Ϊ120)
//         lineinfo[y+4].right-lineinfo[y].right>10/*�ұ��߳�������*/)
//    //     &&abs_m(lineinfo[y - 5].right,lineinfo[y - 4].right)<8
//    //     &&abs_m(lineinfo[y - 6].right,lineinfo[y - 5].right)<8
		if (
          y < 75
         &&((lineinfo[y + 1].right_lost)//�һ��±߲�����,���涪��
		||(lineinfo[y+1].right-lineinfo[y].right>=5*(lineinfo[y+2].right-lineinfo[y+1].right)))
         && !lineinfo[y - 3].right_lost
         && !lineinfo[y - 2].right_lost
         &&!lineinfo[y - 1].right_lost
         &&!lineinfo[y].right_lost
         &&!lineinfo[y + 5].left_lost//��߲�����
         &&!lineinfo[y + 3].left_lost
         &&!lineinfo[y + 2].left_lost
         &&!lineinfo[y + 1].left_lost
         &&!lineinfo[y].left_lost
         &&!lineinfo[y - 5].left_lost
         &&!lineinfo[y - 4].left_lost
         &&!lineinfo[y - 3].left_lost
         &&!lineinfo[y - 2].left_lost
         &&!lineinfo[y - 1].left_lost
         &&y<watch.InLoopAngleR//����־λ�����Ƿ��ѱ����(��պ�ǵ��������Ϊ120)
         &&lineinfo[y+4].right-lineinfo[y].right>10/*�ұ��߳�������*/
         )//�������ԭ���洫
          { 
              watch.InLoopAngleR = y;//��Բ���ĵ�һ���ǵ�������
            //Element=Rifht_ring_confirm;
              if(Element==None) //�ڵ�ǰ��Ԫ��ʱ�������²���������ʱ��ֻ�ҽǵ�
              {
                  right_ring_confirm();       //�����һ�����ȷ��
              }
         }
    }
}
/*****************************************************************************
***@breif	�һ�����ȷ�Ϻ���
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_confirm()
{
	uint8 white_count1=0;	//��һ�ǵ������бȽǵ������С�Ĳ��ֵİ׵��ܸ���
	uint8 white_count2=0;   //��һ�ǵ���һ�бȽǵ������С�Ĳ��ֵİ׵��ܸ���
	uint8 white_count3=0;   //��һ�ǵ�����һ�бȽǵ������С�Ĳ��ֵİ׵��ܸ���
	uint8 black_count=0;    //��һ�ǵ������бȽǵ�������С�Ĳ��ֵĺڵ��ܸ���
	uint8 left_lost=0;      //����Ƿ��߱�־λ
    //for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    for(int y=loop_forward_near;y<95;y++)//����ɨ��(�������洫)
    {
//         if((lineinfo[y].left-lineinfo[y+2].left)>3)//�����ٽ��������ֽϴ�仯
//		 {
//                left_lost=1;//��Ϊ�����ֶ���
//		 }
		 if((lineinfo[y].left-lineinfo[y+2].left>2)//��������洫
			 ||(lineinfo[y+2].left-lineinfo[y].left>4))//�����ٽ��������ֽϴ�仯
		 {
                left_lost+=1;//��Ϊ�����ֶ���
		 }
    }
        if(left_lost<3)//���������ٽ��к����ж�
        {
			for(int x=lineinfo[watch.InLoopAngleR-1].right;x<188;x++)//�����һ���һ���ǵ��ߵĺ�����
            {//�洫�Ѱ׵�ͳ��ע����
				//����ʶ�𵽵��һ���һ�ǵ㸽�����еİ׵�����
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
            for(int y=watch.InLoopAngleR;y>loop_forward_near;y--)//�����һ���һ���ǵ��±ߵ�������
            {
				//ͳ���һ���һ�ǵ������еĺڵ�����
				if(Grayscale[119-y][lineinfo[watch.InLoopAngleR].right]==0)
				{
                     black_count++;
				}
            }
            if((white_count1>=10&&white_count2>=10&&white_count3>=10)&&black_count<10)
				//���д����׵㣬���������ڵ�(�洫û�а׵�ͳ��)
            {
                    //Element=Right_ring;        
                    enter_element(Right_ring);//��ʽ�����һ�Ԫ��
					LEDGREEN_ON;									
//                  if(Element_rem.loop_data[Element_rem.loop_count]==0)//�����С��
//                  {
//                      set_speed(setpara.loop_target_speed);
//                      change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//��ת��PID������Ϊ����ת��PID
//                  }
//                  else//����Ǵ�
//                  {
//                      set_speed(setpara.big_loop_speed);
//                      change_pid_para(&CAM_Turn,&setpara.big_loop_PID);
//                  }
                    begin_distant_integeral(setpara.GoInLoop_DisIntegral+1000);//����·�̻���(����Ҫ�ֶ���4750+)
                    watch.InLoop = 6;//��⵽�һ���һ���ǵ㣬��ʱ������һ���߱���ֱ��
                    return;//�˳����
                }
        }
        Element=None;		//������������������Ϊ����Բ��
        watch.InLoopAngleR=120;//��ʼ����һ�ǵ�λ��
        watch.InLoopAngleL=120;
        return;
}
/*****************************************************************************
***@breif	����һ��뻷��һ���ǵ���Բ��
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_circular_arc()
{
    if(watch.InLoop != 6&&watch.InLoop != 7)//�����һ����뻷�׶�
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
//        if (//(watch.InLoop == 6)&&y <watch.InLoopAngle2 &&
//             y>watch.InLoopAngleR&&//Բ���е��ڵ�һ�ǵ��Ϸ�
//			y <watch.InLoopCirc&&//Բ���е�������ֵ�ں���Χ��
//			!lineinfo[y+3].right_lost&&!lineinfo[y+2].right_lost&&!lineinfo[y+1].right_lost&&
//			!lineinfo[y-2].right_lost&&!lineinfo[y-1].right_lost&&!lineinfo[y-3].right_lost&&
//			//��಻����
//			lineinfo[y+1].right >= lineinfo[y].right&&lineinfo[y+2].right >= lineinfo[y+1].right&&
//			lineinfo[y+3].right <= lineinfo[y].right&&lineinfo[y-1].right >= lineinfo[y].right&&
//			lineinfo[y-2].right >= lineinfo[y-1].right&&lineinfo[y-3].right <= lineinfo[y].right
//			/*�Ҳ�ĳ������������Χ�㣬��Ϊ��Բ����Բ���е�*/
//           )
		 if (//(watch.InLoop == 6)&&y <watch.InLoopAngle2 &&
         watch.InLoopAngleR<55//�뻷�ǵ����괦��һ����Χ��
         &&(y>(watch.InLoopAngleR+20)
		 &&get_integeral_state(&distance_integral)==2)//ǰһ��·�̻������
           &&y <watch.InLoopCirc//Բ���е��ڵ�һ�ǵ��Ϸ�
           &&!lineinfo[y+3].right_lost//�Ҳ಻����(����Բ���м��Բ������)
           &&!lineinfo[y+2].right_lost
           &&!lineinfo[y+1].right_lost
           &&!lineinfo[y-3].right_lost
           &&!lineinfo[y-2].right_lost
           &&!lineinfo[y-1].right_lost
           &&lineinfo[y+1].right >= lineinfo[y].right/*�Ҳ�ĳ������������Χ�㣬��Ϊ��Բ����Բ���е�*/
           &&lineinfo[y+2].right >= lineinfo[y+1].right
           &&lineinfo[y+3].right >= lineinfo[y].right
           &&lineinfo[y-1].right >= lineinfo[y].right
           &&lineinfo[y-2].right >= lineinfo[y-1].right
           &&lineinfo[y-3].right >= lineinfo[y].right
           )//������ԭ���洫
           { 
                watch.InLoopCirc = y;//Բ����͹��(�е�)������
                break;
           }
    }
}
/*****************************************************************************
***@breif	����һ��ڶ����ǵ�(��ڽǵ�)
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_second_angle()
{
    if(watch.InLoop != 6&&watch.InLoop != 7)//�����һ����뻷�׶�
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ����µĵ㣬������ʱ��Ϊ���ף���һ�оݣ�
    {
//        if(watch.InLoopCirc<120&&//�е㴦��δ��ʶ��
//           y<watch.InLoopAngle2&&//��ǰ�㴦��������Χ��
//           watch.InLoopAngle2==120&&//֮ǰ�ڶ��ǵ������ѱ����
//           get_integeral_state(&distance_integral)==2&&//��һ��·�̻��������
//           60<y&&y < (loop_forward_far-2)&&//��ǰ����ͼ���ϰ벿��
//           y>watch.InLoopCirc&&//��ǰ����Բ���е��Ϸ�
//           //&&lineinfo[watch.InLoopCirc].right_lost
//           lineinfo[y+1].right <158&&
//           (lineinfo[y].right-lineinfo[y+1].right)<=2&&//��ǰ���ϱ�����
//           (lineinfo[y-4].right-lineinfo[y].right)>lineinfo[y].right/3/*��ǰ���·����������(����)*/)
		if(watch.InLoopCirc<66//�е㵽��Ļ����
               &&y<watch.InLoopAngle2//��ǰ�㴦��������Χ��
               &&watch.InLoopAngle2==120//֮ǰ�ڶ��ǵ������ѱ����
               &&get_integeral_state(&distance_integral)==2//��һ��·�̻��������
               &&y > 60//��ǰ����ͼ���ϰ벿��
               &&y < (loop_forward_far-2)
               &&y>watch.InLoopCirc//��ǰ����Բ���е��Ϸ�
               &&lineinfo[y+1].right <158
               &&(lineinfo[y].right-lineinfo[y+1].right)<=2//��ǰ���ϱ�����
             &&(lineinfo[y-4].right-lineinfo[y].right)>(187-lineinfo[y].right)/2/*��ǰ���·����������(����)*/
          )//�������ж�����
           {
			   //��¼�һ��ڶ��ǵ�����
				watch.InLoopAngle2 = y;
				watch.InLoopAngle2_x=lineinfo[watch.InLoopAngle2].right;
				break;//�ҵ����˳�
           }
    }
    if(watch.InLoopAngle2!=120&&watch.InLoopAngle2>50)//�󻷵ڶ��ǵ���������ͼ���ϱ�
    {
        find_angle_right_down(watch.InLoopAngle2_x,watch.InLoopAngle2,&watch.InLoopAngle2_x,
		&watch.InLoopAngle2);//�������ҽǵ㲻̫����������õ�
    }
}
/*****************************************************************************
***@breif	�һ���ʼת����
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_begin_turn()
{
    if(watch.InLoop != 6)//�����뻷״̬
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    if(get_integeral_state(&distance_integral)==2&&//·�̻������
       watch.InLoop==6&&
       (watch.InLoopAngle2<=75)/*�һ��ڶ��ǵ������복�Ͻ�*/)//���ﶯ����
    {
        clear_distant_integeral();//���·�̻��ֱ���(׼�����ڻ���) 
		watch.InLoop=7;//���ı�־λ�������뻷����ʱ��ಹ���뻷
		begin_angle_integeral(-setpara.InLoop_AngleIntegral);//��ʼת��ǶȻ���(ע���Ǹ�ֵ)
      //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//��ת��PID������Ϊ����ת��PID
      //set_speed(setpara.loop_target_speed);
      //watch.fix_slope=(float)(188-lineinfo[watch.InLoopAngle2].right)/(115-watch.InLoopAngle2);
    }
}
/*****************************************************************************
***@breif	����С���Ƿ���ȫ���һ�
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_in_loop()
{
    if(watch.InLoop != 7)//���ڲ��������뻷�׶�
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
	
    if( watch.InLoop == 7//���ڽ���Բ���ڲ�·�̻���
        &&get_integeral_state(&angle_integral)==1
        &&get_integeral_data(&angle_integral)<-40/*ת��һ���Ƕ�*/)
        //&&watch.left_near_lost<40
    {
		watch.InLoop = 8;//���ı�־λ����ʱ��ȫ�뻷
		change_trailing_para(Right_ring);//����ѭ������Ϊ�һ��� (����������)    
      //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//��ת��PID������Ϊ����ת��PID
    }
}
/*****************************************************************************
***@breif	С���ǶȻ�����ɣ�׼������
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_prepare_out()
{
    if(watch.InLoop != 8)//С�������뻷״̬
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    if( watch.InLoop == 8&&
        get_integeral_state(&angle_integral)==1&&//���ڽ��л��ڽǶȻ���
        get_integeral_data(&angle_integral)<-180/*�����Ѿ��ﵽһ��ֵ*/
		&&lineinfo[69].left>35
        &&lineinfo[69].left<105)//���������洫ԭ���
   {
       watch.InLoop = 9;//���ı�־λ�������ǻ�����ɣ�׼������
       watch.OutLoop_turn_point_x=lineinfo[69].left;//ת�������꣬���ݸõ���в���(Ϊ����ôȡ)
   }
}
/*****************************************************************************
***@breif	������ʱ��ǵ�λ��
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_out_angle()
{
    if(watch.InLoop != 9)//��ǰ����׼������״̬
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
        if ((watch.InLoop == 9) &&y<80&&
             //lineinfo[y].left_lost
             lineinfo[y + 1].left <= lineinfo[y].left&&
             lineinfo[y+2].left <= lineinfo[y+1].left&&
             lineinfo[y-1].left <= lineinfo[y].left&&
             lineinfo[y-2].left <= lineinfo[y].left&&//��Բ�������ǵ�ĺ�����ֵ���ڸ����ĵ�
             lineinfo[y].left <158&&y>30&&//�ǵ�������ͼ�����
             Grayscale[119-y-2][lineinfo[y].left]==255/*�ǵ��Ϸ���Ϊ�׵�*/)
             {
                 if(watch.OutLoopAngle1>y)//֮ǰδ��⵽�ǵ�(��պ�ǵ��������Ϊ120)
                 {
                 //watch.OutLoopLeft = lineinfo[y].left;
                     watch.OutLoopAngle1 = y; //��¼�����ǵ�������
                 }
             }
   }
}
/*****************************************************************************
***@breif	���һ���ת
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_out_loop_turn()
{
    if(watch.InLoop == 9
       //&&(watch.cross+watch.left_lost)>30//�Ҳඪ�߹��࣬˵������Բ����ֱ�߽��紦
       &&watch.OutLoopAngle1<120
       &&get_integeral_state(&angle_integral)==2//����·�̻����Ѿ����
       &&lineinfo[watch.OutLoopAngle1].left_lost==1/*��ඪ��(����ʱ����Ǻ��ŵ�)*/
       &&watch.OutLoop==0)
    {
        clear_angle_integeral();//��ջ��ֱ�־λ����ֵ
		begin_angle_integeral(setpara.OutLoop_AngleIntegral);//��������ת�����
        begin_distant_integeral(setpara.OutLoop_DisIntegral);//����·�̻��֣���ʱҪ������ת(����Ҫ��)
		//change_trailing_para(None);   //�ص�����Ѳ�ߵĲ���
        watch.OutLoop=1;//������־Ϊ��1�Ѿ�����
    }
}
/*****************************************************************************
***@breif	���Ϊֱ��ʱֱ��
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_out_loop()
{
    if(watch.InLoop != 9&&watch.OutLoop!=1)//��ǰ���ǳ����������״̬
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
//        if((watch.InLoop == 9&&watch.OutLoop==1&&//����ǰ���뻷��־λ�Ƿ���ȷ
//            y<60&&
//            lineinfo[y].left_lost==0&&lineinfo[y+10].left_lost==0&&//��߲��ٶ���
//            ((lineinfo[y+10].left-lineinfo[y].left)*(115-y)/10)<148&&//����������
//            get_integeral_state(&distance_integral)==2)/*����·�̻��������*/)
			if((watch.InLoop == 9
				&&watch.OutLoop==1//����ǰ���뻷��־λ�Ƿ���ȷ
				&&y<60
				&&lineinfo[y].left_lost==0//��߲��ٶ���
				&&lineinfo[y+10].left_lost==0
				&&lineinfo[y].left>10//////////����߲�����Ļ��Ե(���������洫�ӵ�)
				&&lineinfo[y+10].left>10///////
				&&((lineinfo[y+10].left-lineinfo[y].left)*(115-y)/10)<148//�����б�ʴﵽһ��ֵ
				&&lineinfo[y+10].left-lineinfo[y].left>0//������ϵ�������һ����ֵ(������洫��)
				&&get_integeral_state(&distance_integral)==2/*����·�̻��������*/)
                ||(get_integeral_state(&angle_integral)==2/*����ת��ǶȻ��������*/)
				)
                
         {
            clear_distant_integeral();//��ճ���·�̻���
			clear_angle_integeral();//��ճ���ת��ǶȻ���
//          if(Element_rem.loop_data[Element_rem.loop_count]==0)//�����С��
//          {
//              begin_distant_integeral(setpara.loop_out_distance);
//          }
//          else//����Ǵ�
//          {
//              begin_distant_integeral(setpara.big_loop_out_distance);
//          }
			begin_distant_integeral(setpara.LeaveLoop_DisIntegral);//��ʼ�뿪Բ��·�̻���(����Ҫ��)
			//change_trailing_para(None);   //�ص�����Ѳ�ߵĲ���
			watch.InLoop =10;//��Բ����ֱ�У����������ǣ�������ͷ������ȡ����Ԫ�أ�
			watch.OutLoop=1;//��Ҫ����
         }
     }
}
/*****************************************************************************
***@breif	�����һ�����ֱ�ߺ��Ҳ�ǵ�
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_straight_out_angle()
{
	if(watch.InLoop != 10&&watch.OutLoop!=1)//���ǳ�����ֱ�е�״̬
	{
		return;//��ѭ��֮ǰ��������ʡʱ��
	}
    for(int y=loop_forward_near;y<loop_forward_far;y++)//����ɨ��
    {
//        if((watch.InLoop==10)&&watch.OutLoop==1&&//����ǰ���뻷��־λ�Ƿ���ȷ
//          y<watch.OutLoopAngle2&&//���Ͻǵ��־λ�����Ƿ��ѱ����(��պ�ǵ��������Ϊ120)
//          y<80&&y<(watch.watch_lost-10) &&lineinfo[y].left<120&&lineinfo[y].right>60&&//�ǵ㲻����Ļ�ϱ�
//          watch.zebra_flag == 0&&//��ǰ�ް�����
//          (lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30&&
//          (lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30&&
//          (lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30&&
//          lineinfo[y+2].left-lineinfo[y+1].left<5/*�ǵ��Ϸ������ߣ��·����ֶ���*/)
		if((watch.InLoop==10)&&watch.OutLoop==1//����ǰ���뻷��־λ�Ƿ���ȷ
          &&y<(watch.watch_lost-10)//�ǵ㲻����Ļ�ϱ�
          &&y<80
          &&y<watch.OutLoopAngle2//���Ͻǵ��־λ�����Ƿ��ѱ����(��պ�ǵ��������Ϊ120)
          &&lineinfo[y].left<120//�ǵ����������ұ߽�����м�
          &&lineinfo[y].right>60
          //&& watch.zebra_flag == 0
          &&(lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30
          &&(lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30
          &&(lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30
          &&lineinfo[y+2].left-lineinfo[y+1].left<5/*�ǵ��Ϸ������ߣ��·����ֶ���*/
          &&!lineinfo[y].right_lost//�Ҳ���ֶ���
          &&!lineinfo[y+1].right_lost
          &&!lineinfo[y+2].right_lost)//�������ԭ���洫
		 {
			 //��¼��⵽��ֱ���뿪ʱ�ĵ�һ�ǵ�����
             watch.OutLoopAngle2 = y;
		     watch.OutLoopAngle2_x = lineinfo[watch.OutLoopAngle2].right;
         }
    }
}
/*****************************************************************************
***@breif	�һ������ȫ����
***@param	��
***@retval	��
*******************************************************************************/
void right_ring_complete_out()
{
    if (watch.InLoop == 10
		&&get_integeral_state(&distance_integral)==2
		//&&watch.OutLoopAngle2<25
		)  
     {  
         clear_all_flags();                 //�����ɹ�,������б�־
         out_element();                     //�˳�Բ��Ԫ��
		 LEDGREEN_OFF;                      //�ص�
//		 element_number ++;                 //19������״̬��
//		 element_timer_temp = timer.timer_ms;
     //mycar.target_speed=setpara.com_target_speed;//�ָ������ٶ�
		change_trailing_para(None);   //�ص�����Ѳ�ߵĲ���
	 }
}
//�������ҽǵ�
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
//�������ҽǵ�
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
