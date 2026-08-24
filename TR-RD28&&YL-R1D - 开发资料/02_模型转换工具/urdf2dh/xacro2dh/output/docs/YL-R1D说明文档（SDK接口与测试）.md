# YL\-R1D说明文档

# 1 变量及类型

主要增加了两个变量类型：关节Joints、末端Terminal。对应有关节数组JointsList、末端数组。这些变量均在include/**RobSoft/CDataStructure\.hpp**头文件中声明。此外，该文件中同时声明了四元数、向量及相应运算等内容。

此外，其他主要会使用到的变量建议类型为：INT、DOUBLE、BOOL。

## 1\.1 关节 Joints

核心数据为VectorXd类型（在REigen\.hpp中定义）广义关节坐标，为protected类型。public成员函数如下：

1. 构造函数，支持下面几种方式初始化：

    1. 初始化为6自由度关节，默认角度为0（默认）

    2. 指定自由度，默认角度为0

    3. 指定自由度，通过常数初始化角度值

    4. 指定自由度，通过数组初始化角度值

    5. 指定自由度，通过向量初始化角度值

    6. 通过向量初始化角度值，向量长度即为自由度

    7. 通过VectorXd初始化角度值

    此外，拥有析构函数。

2. 关节变量调整函数，包括下面几类：

    1. 赋值函数setValue，与构造函数内容基本一致

    2. append函数，追加关节角

    3. interceptDOF函数，对前若干个关节角进行截取

    4. alignDOF，自由度对齐。

3. 查询函数，主要有：

    1. getValue返回值非void类型，正常查询返回，包括特定关节角double返回、所有关节角VectorXd返回

    2. getValue返回值void类型，传址方式返回，包括数组、向量、VectorXd方式返回所有关节角。

    3. getJointsDOF，查询自由度。

4. 关节空间运算：

    1. norm，计算关节空间二范数，也可以计算两组关节的空间距离

    2. 重载运算\[\]，返回指定关节角度值；

    3. \+、\-重载运算，两组关节相加、减

    4. \*、/重载运算，两组关节一一相乘、除，或关节所有值乘、除以同一常数

    5. ==、！=，判断两组关节相等或不相等

    6. judgeOverMinimum、judgeOverMaximum、judgeABSOverMaximum， 判断是否存在角度小于参考值、大于参考值、绝对值大于参考值

5. 关节值打印函数print，打印所有关节值。

## 1\.2 关节数组 JointsList

核心数据为std::vector\<Joints\>类型存储多个关节，protected类型。使用方法与vector大体相似，不过多赘述。

## 1\.3 末端 Terminal

核心数据包括Point类型末端位置、AttitudeAngle类型末端位姿，为protected类型。public成员函数如下：

1. 构造函数，支持下面几种方式初始化：

    1. 设置默认末端，\(0,0,0,0,0,0\)

    2. 通过Vector6d（在REigen\.hpp中定义）设置位置姿态值

    3. 通过向量设置位置姿态值

    4. 通过6个double类型参数，设置位置姿态值

    5. 通过点和欧拉角设置位置姿态值

    6. 通过点和旋转矩阵设置位置姿态值

    7. 通过齐次矩阵设置位置姿态值

    8. 通过坐标系设置位置姿态值

    此外，拥有析构函数。

2. 关节变量调整函数，包括下面几类：

    1. 赋值函数setValue，与构造函数内容基本一致

    2. append函数，追加关节角

    3. interceptDOF函数，对前若干个关节角进行截取

    4. alignDOF，自由度对齐。

3. 查询函数，主要有：

    1. getValue，返回double类型指定的位置或姿态值，也可以返回Vector6d类型位置位姿

    2. getPoint，返回点位；

    3. getAttitudeAngle、getRotateMatrix、getHomogeneousMatrix，返回欧拉角、旋转矩阵、其次矩阵

4. 关节空间运算：

    1. norm，计算位姿空间二范数，也可以计算两组位姿的空间距离

    2. 重载运算\[\]，返回指定位置或位姿；

    3. \+、\-重载运算，两组位姿相加、减

    4. \*重载运算，两组位姿一一相乘，或位姿乘同一常数

    5. ==、！=，判断两组位姿相等或不相等

    6. judgeOverMinimum、judgeOverMaximum、judgeABSOverMaximum， 判断是否存在位姿小于参考值、大于参考值、绝对值大于参考值

5. 坐标系变换：

    1. getContraryTerminal，当前Terminal表示工件表面向上的坐标系时，获取工具端抓取的Terminal

    2. getTerminalInWorkFrame，已知基坐标系下值，求工件坐标系下的值

    3. getTerminalFromWorkFrame，已知工件坐标系下值，求基坐标系下值

6. 关节值打印函数print，打印所有关节值。

## 1\.4 末端 TerminalList

核心数据为std::vector\<Terminal\>类型存储多个末端，protected类型。使用方法与vector大体相似，不过多赘述。

# 2 控制方法

在编写程序过程中主要依靠**RobotConSys类**实现各种控制，主要由RobotConSys文件夹的头文件声明，包含两个主要部分内容：

1. RobotConSys\_Struct\.h、RobotConSys\_TypeDef\.h定义枚举、结构体及常用数据类型

2. RobotConSys\.h，主要定义了RobotConSys类作为控制接口

下面，对接口进行介绍：

## 2\.0 通用函数参数简介

基本上，所有的函数都包含参数：ROBOTCONSYS\_ARM\_INDEX armIndex 其含义为控制的关节组的编号。

很多地方都会用到vel速度参数，其含义为最大速率的比值

## 2\.1 机器人系统操作

1. 初始化函数init，主要包含：对IP地址与端口的初始化、配置目录设置、Modbus端口。后两个可以选用。是否成功建立联系，可以使用isEstablished函数。

2. 关闭函数close

3. 权限等级查询getAuthority，权限设置函数setAuthority

4. 获取机器人控制系统参数：readRobotConSysPreference、getRobotConSysPreference；设置机器人控制系统参数writeRobotConSysPreference、setRobotConSysPreference

5. 获取/设置机器人参数：上述四个函数中RobotConSysPreference更改为RobotParameter

6. 获取/设置机器人配置：上述四个函数中RobotConSysPreference更改为RobotPreference

7. 获取/设置坐标系配置：上述四个函数中RobotConSysPreference更改为RobotFrame；

8. 修改工具坐标系：modifyTool、modifyToolFrame；修改工件坐标系：modifyWorkFrame、modifyWorkFrame；

9. 标定坐标系：calibrateTCP、calibrateTCFZ、calibrateTCFX、calibrateUSRF

10. 系统状态：机器人伺服状态、错误状态，运动状态，机器人控制系统状态

## 2\.2 运动控制

1. 初始化：searchZero传感器归零；returnZero回到零位；returnHome回到初始位置

2. 点动指令：

    1. jointJOG、terminalJOG、armAngleJOG，关节、末端（会产生严重错误，导致系统断开连接）、臂形角（没效果）点动；stopJOG，停止

    2. startJointsJOGVel、startTerminalJOGVel，关节、末端按一定速率点动；setJOGVel，设置速度；stopJOGVel，停止点动

3. 步进指令：jointStep、terminalStep（极易报错，可能导致系统断开连接）、armAngleStep，关节、末端、臂形角（没效果）步进

4. 关节空间规划关节点列：moveABSJoint，关节空间绝对运动；moveABSJointR，关节空间相对运动（即移动到当前广义坐标\+输入广义坐标点位置）

5. 关节空间规划末端点列：moveJoint，末端空间绝对运动；moveJointR，末端空间相对运动；

6. 末端特定轨迹规划：直线规划（无法使用）、圆弧规划（无法使用）、B样条曲线规划、点列轨迹（点间有平滑处理）、连续轨迹

7. 力矩控制模式：零力拖动示教模式、基于末端力传感器的柔顺控制模式

8. 视觉伺服模式：

9. 夹爪控制：设置状态、更新状态、获取状态

10. 移动底座控制：获取状态、设置控制类型、运动控制

## 2\.3 外部接口

1. 本体IO设备

2. 外部IO设备

3. modbus寄存器

4. 外部TCP设备

5. 程序执行

# 3 其他文件

## 3\.1 REigen\.hpp、CDataStructure\.hpp

主要定义了软件中使用到的数据类型、宏，及一些辅助函数。

## 3\.2 CErrorCode\.hpp

主要定义了错误码，可以快速发现对应问题

## 3\.3 DeviceLayer文件夹

主要定义了设备层的控制交互方式，包括TCP消息交互，以及相机、机器人、车轮底座的交互逻辑方式。



# 4 其他注意事项

## 4\.1 第三方库

主要为Eigen库。

除此之外，需要opencv库。

## 4\.2 链接情况

当前项目将所有的可执行文件编译为\.so类型为主的**共享对象文件**，在执行过程中可能出现无法找到对应库的情况。主要包含下面两类报错：

1. 提示无法找到对应共享文件，例如：

> error while loading shared libraries: libSystemLayer\.so: cannot open shared object file: No such file or directory
> 
> 

2. 共享文件函数无法调用，例如：

可以使用ldd指令，检查是否存在无法查找到的依赖项。

使用ldd指令，检查是否存在无法查找到的依赖项

> CLoadLibrary::openLib failed, \./libRobotConSys\_Client\.so
> 
> load RobotConSys\_Client failed\!
> 
> load func failed: \./RobotConSysDemo: undefined symbol: createRobotConSys\_Client
> 
> 

解决方法有：

1. 将共享文件路径添加到变量LD\_LIBRARY\_PATH中

2. 在程序路径下创建软链接ln \-sf \*\.so 程序路径

# 5 传感器

目前，SDK中能够通过TCP/IP协议直接获取的传感器，不包含IMU、超声波传感器、雷达传感器。视觉图像可以正常读取，但也存在其他读取方式。



# 7 其他参数

机械臂关节范围（应当以度为单位）

|关节编号|最小值|最大值|最大速度|
|---|---|---|---|
|J1|\-150|150|162|
|J2|\-90|105|162|
|J3|\-150|150|324|
|J4|\-90|90|324|
|J5|\-150|150|324|
|J6|\-120|120|324|
|J7|\-360|360|324|

机器人躯体关节范围

|关节编号|最小值|最大值|最大速度|
|---|---|---|---|
|J1（移动关节）|\-300|300|67\.82|
|J2|\-180|180|162|
|J3|\-90|90|324|
|J4|\-90|90|324|

错误码及错误原因

|错误码|错误名称|错误原因|
|---|---|---|
|0|ERROR\_NONE|无错误|
|1|ERROR\_NO\_INVERSE\_KINEMATICS|无逆解|
|2|ERROR\_INVERVE\_KINEMATICS\_OVER\_ITERATION|求解超出最大迭代次数|
|3|ERROR\_OVERRANGE|超出限制位置|
|4|ERROR\_OVERVELOCITY|超出最大速度|
|5|ERROR\_OVERACCELERATION|超出最大加速度|
|6|ERROR\_PARAMETER\_DRIVER\_CHANGE|驱动器参数变更，需重启|
|7|ERROR\_TOOL\_NAME\_INVALID|非法的工具名称|
|8|ERROR\_TOOL\_EXIST|工具已存在|
|9|ERROR\_TOOL\_NOT\_EXIST|工具不存在|
|10|ERROR\_MODIFY\_DEFAULT\_TOOL|默认工具不可修改|
|11|ERROR\_DELETE\_DEFAULT\_TOOL|默认工具不可删除|
|12|ERROR\_WORKFRAME\_NAME\_INVALID|非法的工件名称|
|13|ERROR\_WORKFRAME\_EXIST|工件已存在|
|14|ERROR\_WORKFRAME\_NOT\_EXIST|工件不存在|
|15|ERROR\_MODIFY\_DEFAULT\_WORKFRAME|默认工件不可修改|
|16|ERROR\_DELETE\_DEFAULT\_WORKFRAME|默认工件不可删除|
|17|ERROR\_CALIBRATE\_SAMPLE\_NUM|错误的工具标定采样点数|
|18|ERROR\_SERVO\_OFF|伺服未使能|
|19|ERROR\_SERVO\_ON|伺服已使能|
|20|ERROR\_ROBOT\_DEV|机器人设备报错|
|21|ERROR\_EMERGENCY\_STOP|急停已打开|
|22|ERROR\_CMD\_CONFLICT|指令冲突|
|23|ERROR\_CON\_TRAJ\_NOT\_START|连续轨迹未开始|
|24|ERROR\_CON\_TRAJ\_NOT\_END|连续轨迹未结束|
|25|ERROR\_MOTION\_STOP|运动中止|
|26|ERROR\_NO\_SUCH\_FILE|没有这个文件|
|27|ERROR\_OPEN\_FILE|打开文件失败|
|80|ERROR\_ROBOT\_DEV\_CONNECTION\_INCOMPLETE|机器人设备未连接|
|100|ERROR\_INVALID\_KEYWORD|无效的关键字|
|101|ERROR\_CAMERA\_CONNECT|相机未连接|
|102|ERROR\_IO\_CONNECT|IO未连接|
|103|ERROR\_NO\_FUNCTION|函数不存在|
|104|ERROR\_NO\_MATCH\_FUNCTION|函数不匹配|
|105|ERROR\_SERVER\_POINT\_NULL|服务器指针错误|
|200|ERROR\_UNDEFINE\_VARIABLE|未定义变量|
|201|ERROR\_VARIABLE\_ASSIGNMENT|非法的变量赋值|
|202|ERROR\_POINTER\_ACCESS\_VIOLATION|函数指针访问越界|
|203|ERROR\_SYNTAX\_ERROR|语法错误|
|204|ERROR\_OPERATION\_ASSIGNMENT|不同类型的变量|
|205|ERROR\_NULL\_SENTENCE|空语句|
|206|ERROR\_VARIABLE\_DEFINITION|变量定义错误|
|207|ERROR\_VARIABLE\_REDEFINITION|变量重复定义|
|208|ERROR\_FUNCTION\_REDEFINITION|函数重复定义|
|1900|ERROR\_INVALID\_DEV\_CHANNEL|（无说明）|
|2000|ERROR\_PERMISSION\_DENIED|无权限|

# 8 目前网络配置

当前，将WSL配置网络模式为镜像，IP地址设置为127\.0\.0\.1回环地址。

常用测试指令：

Linux：

nc \-zv 127\.0\.0\.1 8109

curl \-v http://127\.0\.0\.1:8109

Windows：

Test\-NetConnection \-ComputerName 127\.0\.0\.1 \-Port 8102



