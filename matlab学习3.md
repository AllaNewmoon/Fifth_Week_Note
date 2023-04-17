## SerialLink补充
* Clone = SerialLink(twolink, 'name', 'bob')克隆twolink并起名bob。
* 移动基座：p560.base = transl(0, 0, 30*0.0254)
  （移动30英寸高度）
* 对于逆解有多种的情况下，人为指定哪个解
  （eg：qi = p560.ikine6s(T, 'ru')）
  其中l，r表示左右手，u，d表示肘部在上或下，f，n表示手部翻转，不翻转。
* 设定坐标初始值p560.ikine(T, [0 0 3 0 0 0])可以排除一些解
* 将多个机器人关联起来：p8 = platform * p560或p8 = SerialLink([platform, p560])
* platform = SerialLink([0 0 0 -pi/2 1; -pi/2 0 0 pi/2 1],)

### 轨迹规划
* 工作空间可视化：指定q1 = L(1).qlim(1) = rand*(L(1).qlim(2)- L(1).qlim(1));
   其中qlim（1）为最小值，qlim（2）为最大值
```
num = 30000//迭代次数
P = zeros(num, 3);
for i = 1:num
q1 = ...
q = [q1 q2 q3 q4 q5];
T = Five_dof.fkine(q);
P(i,:) = transl(T);
end

plot 3(P(:,1),P(:,2),P(:,3),'b','markersize',1);
```
* 五次多项式轨迹tpoly，混合曲线lspb
```
t = linspace(0,2,51);//时间插值
[P,dP,ddP] = tpoly(0,3,t);
[P,dP,ddP] = lspb(0,3,51,0.1);//0.1是最大速度
```
* mtraj绘制轨迹:[P,dP,ddP] = mtraj(@tpoly/@lspb,[起点],[终点],t)
* mstraj绘制多维多段轨迹:Traj = mstraj(WP, 最大速度, 时间间隔, [初始位置], 插值间隔, 加速时间)
  (WP指要经过的点)
* 位姿插值trinterp(T0, T1, M)
  (eg:trinterp(T1, T2, tpoly(0,2,51)/2))
  注意M必须要在0到1之间
* ctraj位姿插值，默认采用lspb插值

* 轨迹规划旋转部分步骤：由T1，T2求得rpy1，rpy2，再利用mtraj插值，转换为T
* 轨迹规划平移部分操作：transl(T)得到平移部分再进行插值，最后转换为齐次矩阵T
```
rpy1 = tr2rpy(T1)/pi*180;
rpy2 = tr2rpy(T2)/pi*180;

t = linspace(0,2,51);
rpy_traj = mtraj(@tpoly, rpy1, rpy2, t);
T_traj_rot = rpy2tr(rpy_traj);\\旋转部分

P1 = transl(T1);
P2 = transl(T2);
P_Traj = mtraj(@tpoly,P1',P2',t);
T_Traj_transl = transl(P_Traj);//平移部分

n = length(t);
T_traj = zeros(4,4,n);
for i=1:n
    T_Traj(:,:,i) = T_Traj_transl(:,:,i)*T_tran_rot(:,:,i);
end
``` 
* jtraj在关节空间插值：先用ikine解得初末位置关节坐标，再插值 

## 机器人学补充  
### 位形空间
**拓扑等效**：两个表面能连续从一种形状变到另一种形状
不同拓扑的一维空间包括圆，直线，线段。圆可写成S1，表示一维球，直线可写成E1，表示一维欧氏平面。

C-空间可以表示为低维空间的笛卡尔积：
eg：
平面刚体C空间可写为R2*S1
PR机器人为R1*S1
RR机器人为S1*S1 = T2
三维空间刚体：R3*S2*S1

避免奇异值问题的方法：采用多个坐标图，每个图无奇异，或者使用隐式表示，使用高维空间的坐标。

**Pfaffian约束**：对单环，多环机器人，将位形空间隐式表示为θ = [θ1 ... θn]，可以列出闭环方程，对时间求导，能得到A(θ)*θ' = 0,其为Pfaffian约束。不可积的Pfaffian约束为非完整约束。

### 刚体运动：
**角速度与旋转矩阵**：
w可表示为旋转轴和角速度大小的乘积，并且有x' = w X x, y' = w X y, z' = w X z
于是有ri' = w~s~ X ri(i = 1, 2, 3表示x，y，z轴)

写成矩阵形式，有R' = [w~s~Xr1 w~s~Xr2 w~s~Xr3] = w~s~XR
引入运算符w^，将向量变为3*3反对称矩阵（见slam十四讲）。则R' = (w~s~)^R

有结论：trR = 1 + 2cosθ


**运动旋量**：对齐次矩阵T，T^-1^T' = [w~b~^ v~b~,0 0]，w~b~表示物体坐标系下的角速度。将w~b~与v~b~合在一起构成六维向量，称为物体运动旋量V~b~，同理有空间运动旋量。

**伴随变换矩阵**：给定T，其伴随变换矩阵[Ad~r~] = [R 0, p^R R], 有V' = [Ad~r~]V

**螺旋轴**：运动旋量V可写成螺旋轴S与绕该轴转动的速度的复合形式
S可以用{q，s，h}表示，q为轴上任一点，s为螺旋轴方位的单位向量，h为节距，等于螺旋轴方向线速度与角速度的比值
则有V = [sθ' -sθ'Xq + hsθ']
S也可以正则化表示：
1.||w|| = 1，若w = 0，则||v|| = 1, 若w不为0，v = -wXq + hw。
2.S= [w v]

**刚体运动指数坐标**：设S^ = [w^ v, 0 0]，则S^属于se3， 定义指数exp(S^θ) = T = [R p,0 1], log(T) = S^θ

**matlab算法：给定指数坐标找到S与θ**
```
//若R = I，w = 0，v = p/||p||, θ = ||p||
//否则，v = G^-1^(θ)p
function [S, theta] = AxisAng6(expc6)//expc6是6维向量
theta = norm(expc6(1:3));//对前三个（w^θ）求二范数
if NearZero(theta)//相当于w = 0
    theta = norm(expc6(4:6));
end
S = expc6 / theta;
end
```
**计算旋转矩阵对应的对数：**
```
function so3mat = MatrixLog3(R)
acosinput = (trace(R) - 1)/2;
if acosinput >= 1//cos大于1
    so3mat = zeros(3);
elseif acosinput <= 1//trR = -1时，θ = pi
    if NearZero(1 + R(3, 3))
        omg = (1/sqrt(2*(1 + R(3, 3))))...
            *[R(1, 3);R(2, 3);1 + R(3, 3)];
    elseif NearZero(1 + R(2, 2))
        omg = (1/sqrt(2*(1 + R(2, 2))))...
            *[R(1, 2);R(2, 2);1 + R(3, 2)];
    else
        omg = (1/sqrt(2*(1 + R(1, 1))))...
            *[R(1, 1);R(2, 1);1 + R(3, 1)];
    end
    so3mat = VecToso3(pi * omg);
else
    theta = acos(acosinput);
    so3mat = theta * (1/(2 * sin(theta)))*(R - R')
end
end
    
```
**计算齐次矩阵对应的对数：**
```
function expmat = MatrixLog6(T)
[R,p] = TransToRp(T);
omgmat = MatrixLog3(R);//求得旋转部分的w
if isequal(omgmat, zeros(3))//没有旋转部分
    expmat = [zeros(3), T(1:3, 4);0, 0, 0, 0];
else
    theta = acos((trace(R) - 1)/2);
    expmat = [omgmat,(eye(3) - omgmat/2...
            *omgmat * omgmat/theta) * p;
            0, 0, 0, 0 ];//利用公式v = G^-1^(θ)p
end
end
```
（注意分辨S和S^θ， w和w^θ。）
**计算w^θ对应的指数R**：利用指数公式。先求出theta。
```
function R = Matrixexp3(so3mat)
omgtheta = so3ToVec(so3mat);
if NearZero(norm(omgtheta))//没有发生旋转
    R = eye(3)
else
    [omghat, theta] = AxisAng3(omgtheta)//提取w和θ
    omgmat = so3mat / theta;
    R = eye(3) + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
end
end
```
**计算S^θ对应的指数T**：先提取旋转部分求出theta, w^，再利用指数公式

### PoE下的正向运动学
给定初始位姿M，找到每个关节的S与θ。相对世界坐标系则左乘exp(S^θ)，相对末端坐标系则右乘。

**matlab实现：**
```
function T = FKinSpace(M, Slist, thetalist)
T = M;
for i = size(thetalist): -1: 1;
    T = MatrixExp6(VecTose3(Slist(:,1)*thetalist(i))) * T;
end
end
```
其中VecTose3（将六维向量转化为se3矩阵）：的实现如下：
```
function se3mat = VecTose3(V)
se3mat = [VecToso3(V(1:3)), V(4:6);0, 0, 0, 0)];
ebd
```

**从给定的一组关节位置和速度计算末端执行器速度**：

**奇异位形**：正向运动学方程可写为x(t) = f(θ(t))，两边对t求导得v = J(θ)θ'
对于多个J(θ)的线性组合，只要J线性无关，就能生成任意方向的末端速度，但当θ为0或180度时总有J线性相关，称为奇异位形。
（奇异的数学本质是雅可比矩阵不满秩）

对于任意一组关节速度，可以表示为空间中的一个正方形，雅各布矩阵将其映射为速度，表示成平行四边形。也可以映射成单位圆，单位圆映射成末端速度的椭球，称为**可操作度椭球**

**空间雅可比**：由正向运动学的指数积公式可以推导出V~s~ = J~s~(θ)θ'，其将关节速度向量和空间速度联系在一起。J~s~(θ)的第i列物理意义是描述第i个关节轴相对固定坐标系的旋量。J~si~(θ) = [Ad~Ti-1~]*S~i~
同理对于物体坐标系也有V~b~ = J~b~(θ)θ'，

**开链机器人静力学**：若外力旋量-F作用在末端执行器上以平衡关节力矩，则有t = J^T^(θ)F。

**可操作度椭球**：利用θ'^T^*θ' = 1，推出q'^T^A^-1^q' = 1，其中A = JJ^T^为对称正定阵，满足该方程的即为m维空间椭球。记A的特征值为λ~1~ ... λ~m~，特征向量为v~1~ ... v~m~，则主轴方向为v~i~, 长度为sqrt(λ~i~)。
可操作度能定义为：sqrt(λmax/λmin)，越接近1时越趋于球，各向同性。或者定义为sqrt(λ~1~λ~2~...)，此时越大越好。

**matlab实现：根据Slist和thetalist求得J**：
```
function Js = JacobianSpace(Slist, thetalist)
Js = Slist;
T = eye(4);
for i = 2:length(thetalist)
    T = T * MatrixExp6(VecTose3(Slist(:,i - 1) * thetalist(i - 1)));
    Js(:,i) = Adjoint(T) * Slist(:,1);
end
end
```
//adjoint函数把T转化为伴随矩阵Ad~Ti-1~，Slist为S~i~。

### 逆向运动学
**牛顿拉夫森方法**：对于方程g(θ) = 0，取初值θ~0~，将方程泰勒展开，令其等于0解得θ，再代入方程中迭代。
当雅可比矩阵可逆时，可以采用以下方法求解Δθ：

当雅可比矩阵不可逆，用J的伪逆代替J。在matlab中用
```
y = pinv(J) * z
```
求解J*y = z（J不可逆）。

**改进算法**：使其能运用到末端位形为T的情况。
记世界坐标系s，目标d和物体坐标系b，有T~bd~(θ~i~) = T^-1^~sb~(θ~i~)T~sd~ = T~bs~(θ~i~)T~sd~
再利用矩阵对数确定V~b~。
V~b~类比于之前算法中的e = x-f(θ)

**matlab中实现算法**：
```
function [thetalist, success] = IKinBody(Blist, M, T, thetalist0, eomg, ev)
//eomg和ev给出w和v允许的误差范围。
//thetalist0给出初始值
thetalist = thetalist0;
i = 0;
maxiterations = 20;
Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
//先用FKinBody将Blist和thetalist转化为T，进而取对数求得Vb
err = norm(Vb(1:3)) > eomg || normVb((4:6)) > ev;
//判断是否成功，大于允许误差时err为1
while err & i < maxiterations//err为1继续迭代
    thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
    i = i + 1;
    Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
    err = norm(Vb(1:3)) > eomg || normVb((4:6)) > ev;
end
success = ~ err//对err取反
end
```

**逆向速度运动学**：只需要求得关节速度，不需要关节位置。
θ' = pinv(J)V~d~, V~d~表示预期的末端速度，可取T^-1^~sd~(t)T'~sd~(t).