
#include "UI.h"

uint8 Cursor_Postion = 0 ;   //��ʼ�����λ��
uint8 UI_choose_flag = 1 ;
uint8 Car_GO_Flag = 0 ;
uint8 Cursor_Postion_02 = 0 ;

void UI_Show_Frist()
{
    ips200_show_string(Show_startLine,0,"Car_Go");
    ips200_show_string(Show_startLine,20,"Image_show");
    ips200_show_string(Show_startLine,40,"Incremental_PID");
    ips200_show_string(Show_startLine,60,"Data_Save");
}
void UI_Choose()   //ѡ��һ��������
{
    if(key1_flag == 0)
    {
        if(UI_choose_flag == 2)
        {
            ips200_clear();
        }
        UI_choose_flag = 1;
        UI_Show_Frist();

    }
    else   //�����˵�
    {
        if(UI_choose_flag == 1)
        {
            ips200_clear();
        }
        UI_choose_flag = 2;
        if(Cursor_Postion == 0)
        {
            ips200_show_string(Show_startLine,0,"Are you Ready ?");
            if(key2_flag == 1 && UI_choose_flag == 2)
            {
                Car_GO_Flag = 1 ;
                ips200_show_string(Show_startLine,30,"Car GO ");
            }
            if(key2_flag == 0 && UI_choose_flag == 2)
                        {
                            Car_GO_Flag = 0 ;
                            ips200_show_string(Show_startLine,30,"Car STOP ");
                        }
        }
        else if(Cursor_Postion == 1)  //��ʾͼ��
        {
            ips200_show_gray_image(Show_startLine, 0, image_copy[0], LCDW, LCDH, LCDW, LCDH, 0);
            for (uint16 i = 59; i > ImageStatus.OFFLine; i--)

                {


                    ips200_draw_point((uint16)ImageDeal[i].RightBorder+Show_startLine, i, RGB565_RED);//��ʾ��� ��ʾ����

                    ips200_draw_point((uint16)ImageDeal[i].Center+Show_startLine, i, RGB565_BLUE);//��ʾ��� ��ʾ�����
                    ips200_draw_point((uint16)ImageDeal[i].LeftBorder+Show_startLine, i, RGB565_RED);//��ʾ��� ��ʾ�ұ���
                }
            ips200_show_gray_image(Show_startLine, 70, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);

        }
        else if(Cursor_Postion == 2)  //��������ʽPID
        {
            ips200_show_string(Show_startLine,0,"Left_P == ");
            ips200_show_float(Show_startLine+100,0,sptr1.P,5,5);
            ips200_show_string(Show_startLine,20,"Right_P == ");
            ips200_show_float(Show_startLine+100,20,sptr2.P,5,5);
            if(key5_flag ==1 && Cursor_Postion_02 == 0)
            {
                sptr1.P +=0.1;
                key5_flag = 0;
            }
            else if(key5_flag ==1 && Cursor_Postion_02 == 1)
            {
                sptr2.P +=0.1;
                key5_flag = 0;
            }
            else if(key6_flag ==1 && Cursor_Postion_02 == 0)
            {
                sptr1.P -=0.1;
                key6_flag = 0;
            }
            else if(key6_flag ==1 && Cursor_Postion_02 == 1)
            {
                sptr2.P -=0.1;
                key6_flag = 0;
            }
        }
        else if(Cursor_Postion == 3)
        {
            ips200_show_string(Show_startLine,0,"Flash_Save");
            if(key5_flag ==1 && Cursor_Postion_02 == 0)
            {
                Write_Flash();
                ips200_show_string(Show_startLine,100,"Save_Success");
                key5_flag = 0;
            }
        }
    }
}
//�����ʾ
void Cursor_show()
{

    if(UI_choose_flag == 1)
    {
        ips200_show_char(0,Cursor_Postion*Height_data , '>');
        ips200_show_char(7, Cursor_Postion*Height_data, '>');
    }
    else
    {
        ips200_show_char(0,Cursor_Postion_02*Height_data , '>');
        ips200_show_char(7, Cursor_Postion_02*Height_data, '>');
    }

    if (key3_flag == 1)
       {
          ips200_clear();
          if(UI_choose_flag ==1)
          {
              Cursor_Postion+=1;
          }
          else
          {
              Cursor_Postion_02+=1;
          }
                   key3_flag = 0 ;
       }
    if (key4_flag == 1 )
        {
            ips200_clear();
            if(UI_choose_flag==1)
            {
                if(Cursor_Postion>0)
                   {
                       Cursor_Postion-=1;
                   }

            }
            else
            {
                if(Cursor_Postion_02>0)
                  {
                      Cursor_Postion_02-=1;
                  }
            }
                      key4_flag = 0 ;
         }

}

void UI_init()
{
    key_set();
    Cursor_show();
//    Key_Num_show();
    UI_Choose();
}
