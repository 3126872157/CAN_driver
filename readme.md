## 用法
+ 创建CanProtocol对象，填入接口，波特率
+ 注册回调函数：插入自定义消息结构体，注意要使用 struct __attribute__((packed))，填入 can id 和 消息处理函数
+ 发送之前 CanFormat 一下，以便发送多帧拼接的信息
+ 调用 sendMessage 发送
