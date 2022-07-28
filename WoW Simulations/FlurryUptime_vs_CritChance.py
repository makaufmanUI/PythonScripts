import random
import mplcursors
from matplotlib import pyplot
from stuff import clear, fmt, perf

# ********************
SIMULATION_COUNT = 10        # took 187.81 minutes/11,268.67 seconds to run 50,000 simulations each for crit chances from 0% to 80%
# ********************

clear()
pyplot.style.use('ggplot')
mplcursors.cursor(hover=True)
pyplot.xlabel('Crit Chance  (%)', fontweight='bold')
pyplot.ylabel('Time in Flurry  (% of total)', fontweight='bold')
pyplot.title('Time in Flurry  vs.  Crit Chance Percentage', fontweight='bold')


def chance(percent):
    return random.randint(1,100) <= percent
def crit(crit_chance):
    return True if chance(crit_chance) else False

def get_area(y):
    return sum(y)/len(y)*100

def avg_flurry_results(crit_chance):
    total_area = 0
    for i in range(SIMULATION_COUNT):
        total_area += get_area(flurry(crit_chance)[1])
    return total_area/SIMULATION_COUNT

def flurry(crit_chance):
    y = []
    FLURRY = y.append
    is_up, is_down = 1, 0

    swing = 100
    while swing > 0:
        if not crit(crit_chance):
            FLURRY(is_down)
            swing -= 1
        else:
            flurry_swings = 3
            while flurry_swings > 0:
                FLURRY(is_up)
                if crit(crit_chance): flurry_swings = 3
                else: flurry_swings -= 1
    
    x = [x for x in range(len(y))]
    return x,y

def flurry_time(crit_chance):
    flurry_area = avg_flurry_results(crit_chance)
    percent_time_in_flurry = ( (flurry_area * 0.7) / ((flurry_area * 0.7) + (100 - flurry_area)) ) * 100
    return round(percent_time_in_flurry,3)

def varying_crits(lower, upper):
    uptimes = []
    for i in range(lower, upper):
        flurryTime = flurry_time(i)
        print(f"\n> For crit chance of {i}%"+
              f"  -->  {fmt(flurryTime,3,False,True)}% time in flurry")
        uptimes.append(flurryTime)
    print('')
    return uptimes

def find_derivative(uptimes):
    deriv = []
    for i in range(len(uptimes)):
        if i == 0:
            deriv.append(11.628*uptimes[i])
        else:
            deriv.append(11.628*uptimes[i] - 11.628*uptimes[i-1])
    return deriv

@perf
def run():
    uptimes = varying_crits(0,81)
    return uptimes




uptimes = run()
x = [x for x in range(len(uptimes))]
pyplot.plot(x, uptimes)
pyplot.show()


