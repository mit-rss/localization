#set enum(numbering: "i.")

= Lab 5 - Localization
Yohan Guyomard (Group 9)

== Part A
=== Question 1
+ $ T(bold(x)_i=vec(x, y, theta)) = mat(cos(theta), -sin(theta), x; sin(theta), cos(theta), y; 0, 0, 1) $
  $ T_Delta = T(bold(x)_(k-1))^(-1) T(bold(x)_k) 
    = mat(0.999, -0.048, 0.223; 0.048, 0.999, -0.013; 0, 0, 1) $
  
  We can retrieve $Delta bold(x)$ by doing the opposite operation of $T$ (i.e. $x$, $y$ are right-most column and $theta = cos^(-1)(0.997)$)...

  $ Delta bold(x) = mat(0.223, -0.013, 0.0480)^T $
  #linebreak()
+ $ bold(x)_k = T(bold(x)_(k-1)) T(Delta bold(x)) = T(bold(x)_(k-1)) T_Delta = mat(0.457, -0.887, 3.123; 0.887, 0.457, 4.186; 0, 0, 1) $

  Again, retrieve $bold(x)_k$ from the transformation matrix $T(bold(x)_k)$...

  $ bold(x)_k = mat(3.123, 4.186, 1.096)^T $

=== Question 2
For $alpha_"hit" = 0.74$, $alpha_"short" = 0.07$, $alpha_"max" = 0.07$, $alpha_"rand" = 0.12$, $sigma = 0.5 "m"$, $z_"max" = 10 "m"$ and $d = 7 "m"$, we can model the probability $p(z^((i))_k|x_k,m)$ with the following python script:
```py
from math import sqrt, exp, pi

a_hit = 0.74
a_short = 0.07
a_max = 0.07
a_rand = 0.12
sigma = 0.5
z_max = 10
d = 7

def p_hit(z):
    nu = 1
    if 0 <= z <= z_max:
        return nu * (1/sqrt(2*pi*sigma**2)) * exp(-((z-d)**2)/(2*sigma**2))
    return 0

def p_short(z):
    return (2/d) * ((1 - z/d) if 0 <= z < d and d != 0 else 0)

def p_max(z):
    epsilon = 0.1
    return 1/epsilon if z_max - epsilon <= z <= z_max else 0

def p_rand(z):
    return 1/z_max if 0 <= z <= z_max else 0

def p(z):
    return a_hit*p_hit(z) + a_short*p_short(z) + a_max*p_max(z) + a_rand*p_rand(z)

for i, z in enumerate([0, 3, 5, 8, 10]):
    print(f"[{i}] z = {z} -> p(...) = {p(z)}")
```
This gives us the following results:
+ $z^((i))_k = 0 "m" ==> p(z_k^((i))|x_k,m) = 0.032$

+ $z^((i))_k = 3 "m" ==> p(z_k^((i))|x_k,m) = 0.023428571428578904$
+ $z^((i))_k = 5 "m" ==> p(z_k^((i))|x_k,m) = 0.017912354448417746$
+ $z^((i))_k = 8 "m" ==> p(z_k^((i))|x_k,m) = 0.09190663043951833$
+ $z^((i))_k = 10 "m" ==> p(z_k^((i))|x_k,m) = 0.7120000089923066$