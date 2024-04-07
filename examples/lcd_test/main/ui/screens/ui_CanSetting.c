// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.11
// Project name: CANopenUiTest

#include "../ui.h"

void ui_CanSetting_screen_init(void)
{
    ui_CanSetting = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_CanSetting, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Header = lv_obj_create(ui_CanSetting);
    lv_obj_remove_style_all(ui_Header);
    lv_obj_set_width(ui_Header, 320);
    lv_obj_set_height(ui_Header, 50);
    lv_obj_clear_flag(ui_Header, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Header, lv_color_hex(0x323232), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Header, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelCanSetting = lv_label_create(ui_Header);
    lv_obj_set_width(ui_LabelCanSetting, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelCanSetting, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_LabelCanSetting, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelCanSetting, "Settings");
    lv_obj_set_style_text_color(ui_LabelCanSetting, lv_color_hex(0xFBFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelCanSetting, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelCanSetting, &lv_font_montserrat_28, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Body = lv_obj_create(ui_CanSetting);
    lv_obj_remove_style_all(ui_Body);
    lv_obj_set_width(ui_Body, 320);
    lv_obj_set_height(ui_Body, 430);
    lv_obj_set_align(ui_Body, LV_ALIGN_BOTTOM_MID);
    lv_obj_clear_flag(ui_Body, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_LabelBitRate = lv_label_create(ui_Body);
    lv_obj_set_width(ui_LabelBitRate, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelBitRate, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelBitRate, 34);
    lv_obj_set_y(ui_LabelBitRate, 99);
    lv_label_set_text(ui_LabelBitRate, "Bit Rate:");
    lv_obj_set_style_text_font(ui_LabelBitRate, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Roller1 = lv_roller_create(ui_Body);
    lv_roller_set_options(ui_Roller1, "1Mbps\n500kbps\n250kbps\n125kbps", LV_ROLLER_MODE_NORMAL);
    lv_obj_set_height(ui_Roller1, 110);
    lv_obj_set_width(ui_Roller1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_x(ui_Roller1, 55);
    lv_obj_set_y(ui_Roller1, -101);
    lv_obj_set_align(ui_Roller1, LV_ALIGN_CENTER);
    lv_obj_set_style_text_font(ui_Roller1, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_Roller1, lv_color_hex(0xFF0000), LV_PART_SELECTED | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Roller1, 255, LV_PART_SELECTED | LV_STATE_DEFAULT);

    ui_LabelBitRate1 = lv_label_create(ui_Body);
    lv_obj_set_width(ui_LabelBitRate1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelBitRate1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelBitRate1, 35);
    lv_obj_set_y(ui_LabelBitRate1, 192);
    lv_label_set_text(ui_LabelBitRate1, "Node ID:");
    lv_obj_set_style_text_font(ui_LabelBitRate1, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Dropdown1 = lv_dropdown_create(ui_Body);
    lv_dropdown_set_options(ui_Dropdown1, "5\n6\n7\n8\n9");
    lv_obj_set_width(ui_Dropdown1, 140);
    lv_obj_set_height(ui_Dropdown1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown1, 57);
    lv_obj_set_y(ui_Dropdown1, -8);
    lv_obj_set_align(ui_Dropdown1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_set_style_text_font(ui_Dropdown1, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_BtnConnect = lv_btn_create(ui_Body);
    lv_obj_set_height(ui_BtnConnect, 70);
    lv_obj_set_width(ui_BtnConnect, lv_pct(100));
    lv_obj_set_align(ui_BtnConnect, LV_ALIGN_BOTTOM_MID);
    lv_obj_add_flag(ui_BtnConnect, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BtnConnect, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_BtnConnect, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_BtnConnect, lv_color_hex(0xFB051B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnConnect, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_lblConnect = lv_label_create(ui_BtnConnect);
    lv_obj_set_width(ui_lblConnect, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblConnect, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_lblConnect, LV_ALIGN_CENTER);
    lv_label_set_text(ui_lblConnect, "CONNECT");
    lv_obj_set_style_text_font(ui_lblConnect, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_BtnConnect, ui_event_BtnConnect, LV_EVENT_ALL, NULL);

}
