######
#
#   Finds the closest resistance to a given target value using a combination of parallel resistors
#  
######


import time
from itertools import combinations

CommonResistors = [ 10,20,47,100,220,470,1000,2200,4700,10000,22000,47000,100000,220000,470000,1000000  ]

# All possible resistor combinations
# Execution time is roughly 0.00089 seconds
AllCombinations = {
    2: list(combinations(CommonResistors, 2)),  # nChoosek(16,2) =  16! / [2!(16-2)!]  = 120  possible combinations
    3: list(combinations(CommonResistors, 3)),  # nChoosek(16,3) =  16! / [3!(16-3)!]  = 560  possible combinations
    4: list(combinations(CommonResistors, 4)),  # nChoosek(16,4) =  16! / [4!(16-4)!]  = 1820 possible combinations
    5: list(combinations(CommonResistors, 5)),  # nChoosek(16,5) =  16! / [5!(16-5)!]  = 4368 possible combinations
    6: list(combinations(CommonResistors, 6)),  # nChoosek(16,6) =  16! / [6!(16-6)!]  = 8008 possible combinations
}

# All possible equivalent resistor values
# Execution time is roughy 0.0028 seconds
AllEquivalents = {
    2: [round(( 1 / (1/x[0] + 1/x[1]) ),2) for x in AllCombinations[2]],
    3: [round(( 1 / (1/x[0] + 1/x[1] + 1/x[2]) ),2) for x in AllCombinations[3]],
    4: [round(( 1 / (1/x[0] + 1/x[1] + 1/x[2] + 1/x[3]) ),2) for x in AllCombinations[4]],
    5: [round(( 1 / (1/x[0] + 1/x[1] + 1/x[2] + 1/x[3] + 1/x[4]) ),2) for x in AllCombinations[5]],
    6: [round(( 1 / (1/x[0] + 1/x[1] + 1/x[2] + 1/x[3] + 1/x[4] + 1/x[5]) ),2) for x in AllCombinations[6]],
}



# Finds the closest equivalent resistances
# to the target value with 'precision'% error or less
def find(target, n, precision=1):
    return [ round(x,2) for x in AllEquivalents[n] if abs(x - target) < target * precision/100 ]


# Builds a dictionary of lists composed of the
# closest equivalent resistances returned from find()
# Execution time is no longer than 0.001 seconds
def find_all(target, precision=1):
    return { n: find(target, n, precision) for n in range(2,7) }


# Finds the index position of a given resistance
# value in the corresponding AllEquivalents dictionary
# Execution time is roughly 0.0002 seconds per call
def find_index(resistance, n):
    return [ AllEquivalents[n].index(x) for x in AllEquivalents[n] if x == resistance ][0]


# Finds the absolute closest equivalent resistance
# to the target value, once the sets of resistances are known
def find_abs_closest(List, target):
    try: return min(List, key=lambda x: abs(x - target))
    except: return -1


# Prints the closest equivalent resistances for all
# parallel resistor combinations of the given target value
def print_all_equivalents(Dict):
    sum = 0
    print('')
    List = []
    for n in Dict:
        sum += len(Dict[n])
        print(f"\tClosest {n}-resistor values:  ", end='')
        for r in Dict[n]:
            List.append(r)
            print('{0:.2f}'.format(r),"Ω", end='   >   ')
            print(f"{AllCombinations[n][find_index(r,n)]}", end='\n\t\t\t\t    ')
        print('')
    print(f'\n\tTotal # of combinations:  {sum}\n')
    return List


# Main function
def main(target, precision=1):
    start = time.perf_counter()
    print(f'\n\n\n\n\tTarget value:  {target} Ω  ± {precision}%')

    # Underlines the line above this one
    print('\t', end='')
    for i in range(25+len(str(target)))[::-1]:
        if i == 0: print('-')
        else: print('-', end='')
    print('')

    closest_values = find_all(target, precision)
    eqList = print_all_equivalents(closest_values)

    abs_closest_value = find_abs_closest(eqList, target)
    if abs_closest_value == -1:
        print(f'\n\tNo equivalent resistances found within  {precision}%  for target value  {target} Ω\n\n')
        return

    for i in range(2,7):
        if len(closest_values[i]) == 1:
            closest = '{0:.2f}'.format(closest_values[i][0])
            error = round(abs(100-closest_values[i][0]/target*100),3)
            errorf = '{0:.3f}'.format(error)
            print(f'\n\t>>  Closest {i}-resistor combination:  {closest} Ω  with an error of  {errorf}%', end='   >   ')
            print(f'{AllCombinations[i][find_index(closest_values[i][0],i)]}')
        elif len(closest_values[i]) > 1:
            closest = find_abs_closest(closest_values[i], target)
            closestf = '{0:.2f}'.format(closest)
            error = round(abs(100-closest/target*100),3)
            errorf = '{0:.3f}'.format(error)
            print(f'\n\t>>  Closest {i}-resistor combination:  {closestf} Ω  with an error of  {errorf}%', end='   >   ')
            print(f'{AllCombinations[i][find_index(closest,i)]}')

    print('\n\n\t', end='')
    for i in range(31): print('-', end='')
    print(f'\n\tExecution time:  {time.perf_counter() - start:.4f} seconds\n\n')










if __name__ == '__main__':

    again = 'y'
    while again == 'y':
        target = ''
        while not isinstance(target, float):
            target = input('\n\n\n\t>> Enter a target resistance:  ')
            try: target = float(target)
            except: print('\n\n\t> Invalid target resistance value.  Please enter an integer or floating point number.\n\n\n\t')
        
        # check if target input was an int
        test = int(target)
        if not target - test > 0: target = test
        
        precision = ''
        while not isinstance(precision, float):
            precision = input('\n\t>> Enter a precision value as a percentage:  ')
            try: precision = float(precision)
            except: print('\n\n\t> Invalid precision value.  Please enter a number between 0 and 100.\n\n\n\t')
        if precision < 0 or precision > 100:
            print('\n\n\t> Invalid precision value.  Please enter a number between 0 and 100.\n\n\n\t')
            precision = ''
        
        # check if precision input was an int
        test = int(precision)
        if not precision - test > 0: precision = test
        
        main(target, precision)
        again = input('\n\n\t>> Would you like to enter another target value?  (y/n)  ')
        print('')

