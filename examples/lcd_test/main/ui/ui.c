// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.11
// Project name: CANopenUiTest

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
void AnimationSplash_Animation(lv_obj_t * TargetObject, int delay);


// SCREEN: ui_Splash
void ui_Splash_screen_init(void);
void ui_event_Splash(lv_event_t * e);
lv_obj_t * ui_Splash;
lv_obj_t * ui_LabelSplash;


// SCREEN: ui_CanSetting
void ui_CanSetting_screen_init(void);
lv_obj_t * ui_CanSetting;
lv_obj_t * ui_Header;
lv_obj_t * ui_LabelCanSetting;
lv_obj_t * ui_Body;
lv_obj_t * ui_LabelBitRate;
lv_obj_t * ui_Roller1;
lv_obj_t * ui_LabelBitRate1;
lv_obj_t * ui_Dropdown1;
void ui_event_BtnConnect(lv_event_t * e);
lv_obj_t * ui_BtnConnect;
lv_obj_t * ui_lblConnect;
lv_obj_t * ui____initial_actions0;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////
void AnimationSplash_Animation(lv_obj_t * TargetObject, int delay)
{
    ui_anim_user_data_t * PropertyAnimation_0_user_data = lv_mem_alloc(sizeof(ui_anim_user_data_t));
    PropertyAnimation_0_user_data->target = TargetObject;
    PropertyAnimation_0_user_data->val = -1;
    lv_anim_t PropertyAnimation_0;
    lv_anim_init(&PropertyAnimation_0);
    lv_anim_set_time(&PropertyAnimation_0, 3000);
    lv_anim_set_user_data(&PropertyAnimation_0, PropertyAnimation_0_user_data);
    lv_anim_set_custom_exec_cb(&PropertyAnimation_0, _ui_anim_callback_set_opacity);
    lv_anim_set_values(&PropertyAnimation_0, 0, 255);
    lv_anim_set_path_cb(&PropertyAnimation_0, lv_anim_path_linear);
    lv_anim_set_delay(&PropertyAnimation_0, delay + 0);
    lv_anim_set_deleted_cb(&PropertyAnimation_0, _ui_anim_callback_free_user_data);
    lv_anim_set_playback_time(&PropertyAnimation_0, 0);
    lv_anim_set_playback_delay(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_count(&PropertyAnimation_0, 0);
    lv_anim_set_repeat_delay(&PropertyAnimation_0, 0);
    lv_anim_set_early_apply(&PropertyAnimation_0, false);
    lv_anim_start(&PropertyAnimation_0);

}

///////////////////// FUNCTIONS ////////////////////
void ui_event_Splash(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_SCREEN_LOAD_START) {
        AnimationSplash_Animation(ui_LabelSplash, 0);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_CanSetting, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_CanSetting_screen_init);
    }
}
void ui_event_BtnConnect(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        BtnConnectHandler(e);
    }
}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Splash_screen_init();
    ui_CanSetting_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_Splash);
}
