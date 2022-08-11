# Motor kinematics simulation

## Position control

In the following picture, we show a diagram representing the velocity of a motor moving forward from position $x_0$ to position $x_1$, with zero initial velocity $v_0$ at time $t_0$ and acceleration $a$. In a typical situation, the motor accelerates to a maximum velocity $v_{max}$, keeps the same velocity for a while and then decelerates until it reaches the position $x_1$.

The area $\Delta x$ under the curve represents the distance between $x_0$ and $x_1$.

<img src="svg/forward_1.svg">

When

$$\Delta x = x_1 - x_0 \le \frac{v_{max}^2}{a}$$

the velocity trajectory degenerates into a triangle.

<img src="svg/forward_2.svg"><img src="svg/forward_3.svg">

If $x_1< x_0$, the above diagrams become

<img src="svg/reverse_1.svg"><img src="svg/reverse_2.svg"><img src="svg/reverse_3.svg">

If the initial velocity $v_{max} \gt 0$ we have the following situation.

Without knowing the motor trajectory before time $t_0$ we can assume that it reached this point starting from a point $t_{-1}$ and accelerating until point $t_0$.

<img src="svg/forward_translated_1.svg"><img src="svg/forward_translated_2.svg"><img src="svg/forward_translated_3.svg"><img src="svg/forward_translated_4.svg">
