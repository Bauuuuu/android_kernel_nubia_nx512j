#ifndef __ZTEMT_HW_VERSION_H__
#define __ZTEMT_HW_VERSION_H__


typedef enum{
 ZTE_GPIO_PULL_DOWN = 0,//gpio pull down
 ZTE_GPIO_FLOAT,//gpio float
 ZTE_GPIO_PULL_UP,//gpio pull up
 ZTE_GPIO_UNKNOWN,
}ztemt_gpio_status;

typedef enum
{
	HW_A=0,
	HW_B,
	HW_C,
	HW_D,
	HW_E,
	HW_F,
	HW_G,
	HW_H,
	HW_I,
	HW_UN = 200// unknow, fail read
}board_info_type;

typedef struct {
       int inited;
       //adc value, not used yet
	int adc_mv;
       board_info_type adc_index;

      //gpio value for hw
	int gpio_hw_a;
	int gpio_hw_b;
       board_info_type hw_index;

     //gpio value for wwan
	int gpio_wwan_a;
       int gpio_wwan_b;
       int gpio_wwan_c;
       board_info_type wwan_index;
}board_orig_value_st;


typedef struct {
      ztemt_gpio_status gpio1;
      ztemt_gpio_status gpio2;
      char expose_str[20];
}gpio_map_2_st;

typedef struct {
      ztemt_gpio_status gpio1;
      ztemt_gpio_status gpio2;
      ztemt_gpio_status gpio3;
      char expose_str[20];
}gpio_map_3_st;

typedef struct {
    int low_mv;
    int high_mv;
    char expose_str[20];
}adc_map_st;

#endif
