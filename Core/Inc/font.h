#ifndef __FONT_H
#define __FONT_H
#include "stdint.h"
#include "string.h"
typedef struct ASCIIFont {
  uint8_t h;
  uint8_t w;
  uint8_t *chars;
} ASCIIFont;

extern const ASCIIFont afont8x6;
extern const ASCIIFont afont12x6;
extern const ASCIIFont afont16x8;
extern const ASCIIFont afont24x12;

/**
 * @brief 字体结构体
 * @note  字库前4字节存储utf8编码 剩余字节存储字模数据
 * @note 字库数据可以使用波特律动LED取模助手生成(https://led.baud-dance.com)
 */
typedef struct Font {
  uint8_t h;              // 字高度
  uint8_t w;              // 字宽度
  const uint8_t *chars;   // 字库 字库前4字节存储utf8编码 剩余字节存储字模数据
  uint8_t len;            // 字库长度 超过256则请改为uint16_t
  const ASCIIFont *ascii; // 缺省ASCII字体 当字库中没有对应字符且需要显示ASCII字符时使用
} Font;

extern const Font font16x16;

/**
 * @brief 图片结构体
 * @note  图片数据可以使用波特律动LED取模助手生成(https://led.baud-dance.com)
 */
typedef struct Image {
  uint8_t w;           // 图片宽度
  uint8_t h;           // 图片高度
  const uint8_t *data; // 图片数据
} Image;

extern const Image bilibiliImg;
extern const Image IMG00000Img;
extern const Image IMG00001Img;
extern const Image IMG00002Img;
extern const Image IMG00003Img;
extern const Image IMG00004Img;
extern const Image IMG00005Img;
extern const Image IMG00006Img;
extern const Image IMG00007Img;
extern const Image IMG00008Img;
extern const Image IMG00009Img;
extern const Image IMG00010Img;
extern const Image IMG00011Img;
extern const Image IMG00012Img;
extern const Image IMG00013Img;
extern const Image IMG00014Img;
extern const Image IMG00015Img;
extern const Image IMG00016Img;
extern const Image IMG00017Img;
extern const Image IMG00018Img;
extern const Image YUANSHENImg;
#endif // __FONT_H
