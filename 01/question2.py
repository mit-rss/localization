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