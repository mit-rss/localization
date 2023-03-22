import math

# Part 2
a_hit = 0.74
a_short = 0.07
a_max = 0.07
a_rand = 0.12
sigma = 0.5
z_max = 10
d = 7
n = 1
epsilon = 0.1

z_k = 0 #TUNE THIS

def calculate_probability(z_k):
    #p_hit
    p_hit = 0
    if 0 <= z_k and z_k <= z_max:
        p_hit = n * (1/(math.sqrt(2*math.pi*sigma**2))) * math.exp(-1 * (z_k-d)**2/(2*sigma**2))

    #p_short
    p_short = 0
    if 0 <= z_k and z_k <= d and d!= 0:
        p_short = (2/d) * (1 - ((z_k)/(d)))

    #p_max
    p_max = 0
    if z_max - epsilon <= z_k and z_k <= z_max:
        p_max = 1/epsilon

    #p_rand
    p_rand = 0
    if 0 <= z_k and z_k <= z_max:
        p_rand = 1/z_max

    #Combined Probability
    p_total = a_hit * p_hit + a_short * p_short + a_max * p_max + a_rand * p_rand

    return p_total

# 0 m : 0.032
# 3 m : 0.023428571428578904
# 5 m : 0.017912354448417746
# 8 m : 0.09190663043951833
#10 m : 0.7120000089923066