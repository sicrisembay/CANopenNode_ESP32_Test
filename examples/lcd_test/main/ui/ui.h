// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.11
// Project name: CANopenUiTest

#ifndef _CANOPENUITEST_UI_H
#define _CANOPENUITEST_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

#include "ui_helpers.h"
#include "ui_events.h"

void AnimationSplash_Animation(lv_obj_t * TargetObject, int delay);
// SCREEN: ui_Splash
void ui_Splash_screen_init(void);
void ui_event_Splash(lv_event_t * e);
extern lv_obj_t * ui_Splash;
extern lv_obj_t * ui_LabelSplash;
// SCREEN: ui_CanSetting
void ui_CanSetting_screen_init(void);
extern lv_obj_t * ui_CanSetting;
extern lv_obj_t * ui_Header;
extern lv_obj_t * ui_LabelCanSetting;
extern lv_obj_t * ui_Body;
extern lv_obj_t * ui_LabelBitRate;
extern lv_obj_t * ui_Roller1;
extern lv_obj_t * ui_LabelBitRate1;
extern lv_obj_t * ui_Dropdown1;
void ui_event_BtnConnect(lv_event_t * e);
extern lv_obj_t * ui_BtnConnect;
extern lv_obj_t * ui_lblConnect;
extern lv_obj_t * ui____initial_actions0;





LV_FONT_DECLARE(ui_font_MicrogrammaPt28);



void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
