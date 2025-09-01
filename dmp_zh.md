# DMP -- Dynamical Movement Primitives
- [1] Ijspeert A J , Nakanishi J , Hoffmann H ,et al.Dynamical movement primitives: learning attractor models for motor behaviors[J].Neural computation, 2013, 25(2):328-73.DOI:10.1162/NECO_a_00393.
- [2] https://studywolf.wordpress.com/2013/11/16/dynamic-movement-primitives-part-1-the-basics/
- [3] https://studywolf.wordpress.com/2016/05/13/dynamic-movement-primitives-part-4-avoiding-obstacles/

## 1. DMP -- Canonical System
 
$$ 
\tau \dot{x} = -\alpha_x x 
$$

上式是 **一个一阶线性衰减动力学**，它的来源和作用一般是作为“canonical system”（规范系统）的形式引入的。这个写法常见于 **Dynamic Movement Primitives (DMP)** 等运动生成框架。  

---

#### 推导思路 && 说明

1. **目标：用一个动力学变量代替时间**  
   - 在很多运动规划问题里，不希望轨迹显式依赖于时间 $t$，而是引入一个内部状态变量 $x(t)$，它从 **1 衰减到 0**，相当于“隐式的时钟”。  
   - 这样做的好处是：轨迹可以自然地缩放到不同的持续时间 $\tau$，而不用改函数定义。

2. **最简单的一阶系统：指数衰减**  
   - 设想我们需要一个单调递减的变量 $x(t)$，可以选用指数形式：

$$
     x(t) = e^{-\frac{\alpha_x}{\tau} t},
        \quad x(0)=1, \quad 
        \lim_{t \to \infty} x(t)=0
     $$
     
   - 这种函数天然满足“随时间衰减”的需求。

3. **对应的微分方程**  
   - 对 $x(t)$ 求导：
    
$$ \dot{x}(t) = -\frac{\alpha_x}{\tau} x(t) $$

   - 两边乘以 $\tau$：
   
$$
     \tau \dot{x}(t) = -\alpha_x x(t)
     $$
     
   - 这就是论文[1]中看到的方程(2.2)。

4. **变量解释**
    - $\tau$：时间缩放因子(trajectory duration)，可以拉伸/压缩轨迹的时间尺度。  
    - $\alpha_x$：衰减速率参数，决定 $x$ 收敛到 0 的快慢。  
  

5. **代码实践** 
    - 关于规范函数的原理基本实现可以查看 class CanonicalSystem
    - 需要说明的是，在本项目代码实际编写中，修改了论文式子(2.2)，将其修改成
        
$$
   \dot{x}(t) = -\tau \alpha_x x(t)
   $$  
   
   - 上式在函数 CanonicalSystem::generateCanonicalSystemDiscreteTrack() 中实现。$\tau$ 越大，时间长度越短。
   - 如果需要查看生成的规范函数图，可以 
        ```bash
        cd build && ./canonical_system 

7. **物理意义**
   - 规范函数不是描述物理量的真实动力学，而是一个**虚拟时钟**，用来驱动后续的非线性轨迹生成。  

#### 总结
这个方程不是“推导出来的自然规律”，而是**人为设计**的一个“时间替代变量”的一阶线性系统。它的解就是指数衰减，方便控制轨迹的进展。

---

## 2. DMP -- Gaussian Function

在 **DMP（Dynamic Movement Primitive）** 中，高斯基函数用于构造非线性 **forcing term**，以便学习和重现复杂轨迹。其基本思想是将轨迹表示为加权高斯基函数的线性组合。



1. **相位变量**

DMP 使用一个 **相位变量 $x$** 替代时间，用来保证运动随时间自动衰减：

$$ \tau \dot{x} = -\alpha_x x $$

- $\tau$ ：运动时间常数  
- $\alpha_x$ ：衰减系数  
- 初始条件 $x(0) = 1$，随着时间衰减到 0

> 这样，无论运动执行多长时间，轨迹的非线性基函数总能按相同比例激活。

2. **高斯基函数定义**

高斯基函数通常定义在相位变量 $x$ 上：

$$
\psi_i(x) = \exp\Big(-h_i (x - c_i)^2\Big)
$$

- $c_i$ ：第 $i$ 个基函数的中心  
- $h_i$ ：宽度系数（决定基函数的覆盖范围），也就是 $ \frac{1}{2\sigma^2} $  
- $i = 1, 2, \dots, N$  

> 注意：这里的高斯函数不是以时间为中心，而是以相位变量 $x$ 为中心，这样可以保证运动的时间缩放性。

3. **基函数中心与宽度选择**

- **中心 $c_i$**  
规范函数并非是线性函数，而是非线性函数，如果直接在 x-space $x \in [0,1]$ 上均匀分布高斯中心 $c_i$，会使得输出的高斯函数图 在初始阶段快速扫过 中心点，后期中心点又会变得稀疏，相当于把 Canonical System 展开铺平看。
因此先选择 t-space 时间上的均匀分布，再映射到 相位变量 $x $，也就是

$$
  c_i = x(t = \frac{i-1}{N-1}) = e^{ \Big( - \tau \alpha_x \frac{i-1}{N-1}\Big)}
  $$
  
其中 N 是指 基函数的个数。

#### 高斯函数中心点小结
- 直接在 $x$ 上均匀分布会导致时间上的基函数分布不均匀  
- 采取在时间上均匀分段，把每个时间点映射到 $x(t)$，用映射后的 $x$ 作为高斯中心

这样可以保证：

- 轨迹在整个时间段内的基函数激活较均匀  
- 前期和后期都能有效覆盖轨迹变化  

4. **宽度 $h_i$**  
  按照 blog[2] 将宽度设置为 
 
$$
  h_i = \frac{\frac{N^{1.5}}{c_i}} { \alpha_x}
  $$

> 这种设计保证基函数在相位变量衰减过程中有适当重叠，从而平滑表示非线性项。

---
## 3. DMP -- Nonlinear Target Force Term

**Question:** 为什么 $f_d(t)=\tau^2 \ddot y_d(t)-\alpha_y\big(\beta_y(g-y_d(t))-\tau \dot y_d(t)\big)$ 成立？

1. **从 DMP 的动力学出发**
常见的一维 DMP 形式（向量情形分量独立同理），在论文[1] 式子(2.1)：

$$
\begin{aligned}
\tau \dot y &= z, \\
\tau \dot z &= \alpha_y\big(\beta_y (g - y) - z\big) + f(x).
\end{aligned}
$$

**含义**
- **$y(t)$**：系统的**位置**（position），也是要模仿/生成的轨迹量  
  - 例：末端位姿的某个坐标、关节角度等。

- **$z(t)$**：**辅助变量**，等价于**速度的缩放版**  
  - 由第一式可得：$\;z = \tau \dot y\;\Rightarrow\;\dot y = z/\tau$。

**Question:** 为什么要引入 $z$
- 将原本的二阶系统

$$
  \tau^2 \ddot y \;=\; \alpha_y\big(\beta_y(g-y)-\tau\dot y\big) + f(x)
  $$
  
  拆成两个一阶方程，便于数值积分与稳定性分析。
- 在此框架下，$f(x)$ 作为**非线性 forcing term**，为轨迹提供形状灵活性；$\alpha_y,\beta_y$ 提供类似弹簧-阻尼的稳定收敛特性。

2. **合并为关于 $y$ 的二阶方程**
由第一式 $z = \tau \dot y$，对时间再求导：

$$
\dot z = \tau \ddot y.
$$

将它代入第二式（并把 $f$ 写成 $f(x)$ 简记为 $f$）：

$$
\tau(\tau \ddot y) \;=\; \alpha_y\big(\beta_y (g - y) - z\big) + f.
$$

用 $z=\tau \dot y$ 代回：

$$
\tau^2 \ddot y \;=\; \alpha_y\big(\beta_y (g - y) - \tau \dot y\big) + f.
$$

4. **解出非线性项 $f

$$
\boxed{\,f \;=\; \tau^2 \ddot y \;-\; \alpha_y\big(\beta_y (g - y) - \tau \dot y\big)\, }.
$$

5. **用演示数据得到 $f_d(t)$**
参考论文[1]式子(2.11),(2.12):
在**学习阶段**，把 $y, \dot y, \ddot y$ 替换为演示轨迹的 $y_d, \dot y_d, \ddot y_d$：
$$
\boxed{\,f_d(t) \;=\; \tau^2 \ddot y_d(t) \;-\; \alpha_y\big(\beta_y (g - y_d(t)) - \tau \dot y_d(t)\big)\, }.
$$

这就是代码里
$$
\texttt{f\_target[g] = }\tau^2 \ddot y_d[g] - \alpha_y\big(\beta_y (g - y_d[g]) - \tau \dot y_d[g]\big)
$$
的来源。随后用高斯基函数拟合 $f_d(t)$ 得到权重 $w_i$，在**再现阶段**用
$$
f(x) \approx \frac{\sum_i \psi_i(x) w_i}{\sum_i \psi_i(x)} \, x (g - y_0)
$$
驱动系统，从而产生与演示相似且可迁移（改 $g,\tau$ 仍稳定）的轨迹。

---

## 4. DMP -- Weight
在 DMP 中，非线性 forcing term $ f(x) $ 用基函数展开形式表示：

$$
f(x) = \frac{\sum_i \psi_i(x) \, w_i}{\sum_i \psi_i(x)} \, x \, (g - y_0)
$$

其中：

- $\psi_i(x) = \exp\!\left(-\frac{1}{2\sigma_i^2}(x - c_i)^2\right)$ 为高斯基函数  
- $w_i$ 为基函数对应的权重  
- $c_i, \sigma_i$ 分别是基函数的中心和宽度  
- $g, y_0$ 分别为目标点和起始点  

权重 $w_i$ 通过“逐基函数的局部加权回归”求解：

$$
w_i = \frac{\sum_t \psi_i(x_t) \, x_t (g - y_0) \, f_d(t)}
           {\sum_t \psi_i(x_t) \, \big(x_t (g - y_0)\big)^2}
$$

其中 $ f_d(t) $ 是根据示教轨迹计算得到的理想 forcing term。

因此在 DMP 中，逐基函数求权重 $w_i$ 可以看作一个 **加权线性回归** 问题。 
详细实现可以看 论文[1] 式子(2.13) (2.14) 

---


## 5. DMP -- Nonlinear Forcing Term

DMP 的非线性项表示为加权高斯函数的组合：

$$
f(x) = \frac{\sum_{i=1}^{N} \psi_i(x) w_i}{\sum_{i=1}^{N} \psi_i(x)} x (g - y_0)
$$

- $w_i$ ：学习得到的权重  
- $y_0$ ：初始位置  
- $g$ ：目标位置  
- $x (g - y_0)$ ：缩放因子，保证运动幅度可缩放  
  详情可以查看 论文[1] 式子(2.18) (2.19)

---

## 6. DMP -- Generate DMP Trajectory

在离散 DMP 中，加速度、速度和位置的更新公式为：
**加速度更新**：
$$
\ddot{y}[t] = \alpha_y \big(\beta_y (g - y[t-1]) - \dot{y}[t-1] \big) + f(t)
$$

**速度更新**：
$$
\dot{y}[t] = \dot{y}[t-1] + \tau \, \ddot{y}[t] \, \Delta t
$$

**位置更新**：
$$
y[t] = y[t-1] + \tau \, \dot{y}[t] \, \Delta t
$$

其中：

- $\tau$：时间缩放因子  
- $\Delta t$：时间步长  
- $\ddot{y}[t]$：当前步加速度  
- $\dot{y}[t]$、$y[t]$：当前步速度和位置


此外 $\ddot{y}[t]$ 可以分为两部分：

1. **线性二阶系统部分（收敛终点）**  
$$
\alpha_y \big(\beta_y (g - y[t-1]) - \dot{y}[t-1] \big)
$$  
- 类似阻尼弹簧系统  
- 作用：让轨迹自然收敛到目标点 $g$  
- 如果没有 $f(t)$，轨迹将平滑单调地收敛

2. **非线性 forcing term（轨迹再现）**  
$$
f(t)
$$  
- 修正轨迹，使其贴合示教轨迹的非线性形状  
- 保证生成轨迹不仅收敛，还能重现复杂动作

**总结**：  

> 前半部分 = “收敛控制”，  
> 后半部分 = “轨迹再现/细节修正”

---

## 7. DMP -- Obstacle
- 详情查看 blog[3] + 代码注释


