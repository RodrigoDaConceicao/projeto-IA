def trapezoidal(x, a, b, c, d):
    if x <= a or x >= d:
         return 0.0
    elif b <= x <= c:
            return 1.0
    elif a < x < b:
         return (x - a) / (b - a)
    else:  
            return (d - x) / (d - c)


def triangular(x, a, b, c):
    if x <= a or x >= c:
        return 0.0
    elif x == b:
        return 1.0
    elif a < x < b:
        return (x - a) / (b - a)
    else:  # b < x < c
        return (c - x) / (c - b)


def fuzzy_dist_cubo(d):
    return {
        "perto": trapezoidal(d, 0.0, 0.0, 0.15, 0.30),
        "medio": triangular(d, 0.20, 0.45, 0.70),
        "longe": trapezoidal(d, 0.55, 0.75, 1.0, 1.0)
        }
    

def fuzzy_ang_cubo(a):
    return {
        "esquerda": trapezoidal(a, -math.pi, -math.pi, -0.5, -0.1),
        "centro": triangular(a, -0.2, 0.0, 0.2),
        "direita": trapezoidal(a, 0.1, 0.5, math.pi, math.pi)
        }


def fuzzy_dist_obs(d):
    return {
        "perto": trapezoidal(d, 0.0, 0.0, 0.20, 0.40),
        "medio": triangular(d, 0.30, 0.55, 0.80),
        "longe": trapezoidal(d, 0.65, 0.85, 1.0, 1.0)
        }


def AND(a, b):
    return min(a, b)


   
def OR(a, b):
    return max(a, b)
    


def defuzzify(fuzzy_out, values):
    num = sum(fuzzy_out[k] * values[k] for k in fuzzy_out)
    den = sum(fuzzy_out[k] for k in fuzzy_out)
    return num / den if den > 0 else 0.0