#ifndef CUSTOM_UI_DRAW_H
#define CUSTOM_UI_DRAW_H

#include "main.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "string.h"
#include "cmsis_os.h"

#ifndef PI
#define PI					3.14159265358979f
#endif

/*���ȶ���*/
#define MAX_SIZE 128
#define frameheader_len 5
#define cmd_len 2
#define crc_len 2

/**
 * @brief ����������ID
 * @param SENDER_ID   R3 3        ���� 103
 * @param RECEIVER_ID �췽 0x0103   ���� 0x0167
 */
#define SENDER_ID 103
#define RECEIVER_ID 0x0167

typedef struct
{
  uint16_t gx;
  uint16_t gy;
  float angle;
  float r;
} tank_ui_t;

typedef struct
{
  uint16_t aimy;
  int16_t angle;
} aim_dynamic_t;

typedef struct
{
  float power_ratio;
  uint16_t jindu;
  uint16_t voltage;
} super_capacity_t;

#if 0 //֡ͷ
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;
#endif

/**
  *@brief �������ݶ�ͷ��Ϣ
  *@param data_cmd_id:   ����id(ɾ��������x��ͼ�Σ������ַ�)
          this parammeter can be any combination of the folowing values:
                0x0200~0x02FF   ���������˼�ͨ��
                0x0100          �ͻ���ɾ��ͼ��
                0x0101          �ͻ��˻���һ��ͼ��
                0x0102          �ͻ��˻��ƶ���ͼ��
                0x0103          �ͻ��˻������ͼ��
                0x0104          �ͻ��˻����߸�ͼ��
                0x0110          �ͻ��˻����ַ�ͼ��
  *@param sender_ID:     ������id����У��
          this parammeter can be any combination of the folowing values:
                1��Ӣ��(��)��2������(��)��3/4/5������(��)��6������(��)��7���ڱ�(��)��9���״�վ���죩
                101��Ӣ��(��)��102������(��)��103/104/105������(��)��106������(��)��107���ڱ�(��)�� 109���״�վ������
  *@param receiver_ID:   ������id����У��
          this parammeter can be any combination of the folowing values:
                �ͻ��� ID��0x0101 ΪӢ�۲����ֿͻ���(��)��0x0102�����̲����ֿͻ���((��)��0x0103/0x0104/0x0105��
                ���������ֿͻ���(��)��0x0106�����в����ֿͻ���((��)�� 0x0165��Ӣ�۲����ֿͻ���(��)��0x0166����
                �̲����ֿͻ���(��)��0x0167/0x0168/0x0169�����������ֿͻ��˲���(��)��0x016A�����в����ֿͻ��� (��)��
  */
typedef __packed struct
{
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/**
 * @brief ͼ�����ݽṹ��
 *        ��3��u8��3��u32����u32λ���в����������λ�Σ���ֱ�Ӹ�ֵ��������λ
 * @param operate_type  ͼ�β���    bit0-2:   0 �ղ�����1 ���ӣ�2 �޸ģ�3 ɾ��
 * @param graphic_tpye  ͼ�����    bit3-5��  0��ֱ�ߣ�1�����Σ�2����Բ��3����Բ��4��Բ����5����������6����������7���ַ���
 * @param layer         ͼ����      bit6-9:   0~9
 * @param color         ��ɫ        bit10-13  0��������ɫ��1����ɫ��2����ɫ��3����ɫ��
 *                                            4���Ϻ�ɫ��5����ɫ��6����ɫ��7����ɫ��8����ɫ��
 * @param start_angle   ��ʼ�Ƕ�    bit14-22   0~360
 * @param end_angle     ��ֹ�Ƕ�    bit23-32   0~360
 * --------------------------------------------------
 * @param width         �߿�        bit0-9
 * @param start_x                   bit10-20
 * @param start_y                   bit21-31
 * ----------------------------------------------------
 * @param radius        �����С��뾶 bit0-9
 * @param end_x                     bit10-20
 * @param end_y                     bit21-31
 */
typedef __packed struct
{
  uint8_t graphic_name[3];
  uint32_t operate_tpye : 3;
  uint32_t graphic_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t radius : 10;
  uint32_t end_x : 11;
  uint32_t end_y : 11;
} graphic_data_struct_t;

/**
 * @brief ���Ƹ�������������
 * @param data: ���32λ������λ
 *
 */
typedef __packed struct
{
  uint8_t graphic_name[3];
  uint32_t operate_tpye : 3;
  uint32_t graphic_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t data : 32;

} Data_graphic_data_struct_t;

/**
 * @brief �ͻ��˻���һ��ͼ��
 */
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;
/**
 * @brief ����2��ͼ��
 * @brief cmd : 0x0102
 */
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

/**
 * @brief ����5��ͼ��
 *
 */
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

/*������*/
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  Data_graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t_data;

/*********�ͻ��˻���7��ͼ��*****/
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

//�����ַ�
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  graphic_data_struct_t grapic_data_struct;
  uint8_t data[30];
} ext_client_custom_character_t;

/**
 * @brief ɾ��ͼ��
 * @param operate_type:  0 �ղ��� ,1 ɾ��ͼ�� ��2 ɾ������
 * @param layer �� ͼ�� 0~9
 */
typedef __packed struct
{
  ext_student_interactive_header_data_t header_data_t;
  uint8_t operate_tpye;
  uint8_t layer;
} ext_client_custom_graphic_delete_t;

//-----------------------------------------------------------------------------------------------//

void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len);

/*sender_id 1  receiver id 0x0101*/
void test_7_graphic(uint16_t sender_id, uint16_t receiver_id);
void Send_Speed(uint16_t sender_id, uint16_t receiver_id);

/***********ɾ��ͼ��*************/
void clear_all_layer(uint16_t sender_id, uint16_t receiver_id);
void clear_single_layer(uint16_t sender_id, uint16_t receiver_id, uint8_t layer);

/***********��������**************/

void draw_single_float(uint16_t sender_id, uint16_t receiver_id, float data);
void draw_single_integral(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x, uint16_t start_y, int32_t data, uint16_t op_type);

/**********����ͼ��*****************/
void draw_single_line(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x,
                      uint16_t start_y, uint16_t end_x, uint16_t end_y,
                      uint8_t color, uint8_t layer, uint8_t width, uint16_t op_type);

void draw_single_circle(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x, uint16_t start_y,
                        uint16_t radius, uint8_t color, uint8_t layer, uint8_t width, uint16_t op_type);

void draw_single_ellipse(uint16_t sender_id, uint16_t receiver_id, uint16_t start_x, uint16_t start_y,
                         uint16_t end_x, uint16_t end_y, uint8_t color, uint8_t layer, uint8_t width);

void draw_single_arc(uint16_t sender_id, uint16_t receiver_id, uint16_t start_angle, uint16_t end_angle,
                     uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y,
                     uint8_t color, uint8_t layer, uint8_t width);
/**
 * @brief test for single graphic
 * @param graphic name start with 1
 */
void draw_single_line_test(uint16_t sender_id, uint16_t receiver_id,uint16_t op_type);
void draw_single_tangle_test(uint16_t sender_id, uint16_t receiver_id,uint16_t op_type);








/*******************��̬ui*********/
void draw_dynamic_ui(uint16_t sender_id, uint16_t receiver_id, uint16_t origin_x,
                     uint16_t origin_y, uint16_t dx, uint16_t dy, uint16_t ddx, uint16_t ddy, uint16_t op_type);

/****************Application*******************/
void draw_init_all(uint16_t sender_id, uint16_t receiver_id);
void draw_chassis_mode(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);       // $3
void draw_chassis_mode_state(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type); // $1
void draw_gimbal_mode(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);
void draw_aim_assitant1(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type, uint16_t layer); // $2
void draw_aim_assitant2(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type, uint16_t layer); // $7
void draw_chassis_run_bordor(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);           // $4
void draw_gimbal_relative_angle_tangle(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);// $6
void cal_draw_gimbal_relative_angle_tangle(tank_ui_t *tank); //����Ƕ�
void draw_state_of_top(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type); //$8
void draw_state_of_top_circle(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type); //$9
void draw_capacity_number(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type); // $10
void draw_capacity_char(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type); //
void draw_capacity_tangle(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);
void cal_draw_pitch_aim(aim_dynamic_t *aim_dynamic);
void audio_enemy_godlike(uint16_t sender_id, uint16_t receiver_id);
void cal_capacity(super_capacity_t * super_capacity);
void draw_get_robot_id(void);
void draw_servo_bull(uint16_t sender_id, uint16_t receiver_id,uint16_t op_type);
void draw_servo_bull_circle(uint16_t sender_id, uint16_t receiver_id,uint16_t op_type);
void draw_pitch(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);
void draw_pitch_number(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);
void draw_speed_cut(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);
void draw_speed_cut_number(uint16_t sender_id, uint16_t receiver_id, uint16_t op_type);
void draw_top_speed(uint16_t sender_id, uint16_t receiver_id,uint16_t op_type);
#endif
