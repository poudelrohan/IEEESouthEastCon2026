# Color Recognition IIC Registers (颜色识别IIC寄存器)

## Device Address: 0x52 (设备地址：0x52)



- ### Color Recognition (颜色识别)

  | Register Address (寄存器地址) | Data Format (数据格式)(unsigned char)                    |
  | :--------: | :----------------------------------------------------------: |
  |    0x00    | data[0]: Red center X coordinate (红色中心X轴坐标)<br/>data[1]:Red center Y coordinate (红色中心Y轴坐标)<br/>data[2]:Red detection box width (红色检测框宽度)<br/>data[3]:Red detection box height (红色检测框长度)<br/> |
  |    0x01    | data[0]:Blue center X coordinate (蓝色中心X轴坐标)<br/>data[1]:Blue center Y coordinate (蓝色中心Y轴坐标)<br/>data[2]:Blue detection box width (蓝色检测框宽度)<br/>data[3]:Blue detection box height (蓝色检测框长度)<br/> |
  
  
  
  