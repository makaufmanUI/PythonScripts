#####
#
#     Hanldes queries to google ngrams, returns a timeseries of the usage
#       frequency in the google books corpus from 1800-2019 as a line chart
#
#####


import json
from pathlib import Path
from secrets import choice
from matplotlib import pyplot
from urllib.request import urlopen



def raiseException(e, line, file=None):
    """
    Raises an exception (prints to console).
    - `e`: exception
    - `line`: line number
    - `file`: file name (optional, default is None)
    """
    if file is None:
        print(f"\n>>>  Exception at line {line}:")
        for i in range(len(str(line))+24): print('-',end='')
        print(f"\n{e}")
    else:
        file = file.split("\\")[-1]
        print(f"\n>>>  Exception at line {line} in {file}:")
        for i in range(len(str(line))+len(file)+28): print('-',end='')
        print(f"\n{e}")




#*************************************************************#
#                                                             #
#   Given a raw discord message,                              #
#   extracts the list of games within,                        #
#   and chooses one of them at random.                        #
#     Input:  raw discord message beginning with ".",         #
#             and containing the word "game" within it:       #
#             ".game", ".pickgame", ".choosegame", etc.       #
#     Output: a randomly chosen game from the list provided   #
#                                                             #
#   Example message: .pickgame halo, phasmo, bif, rocket      #
#                                                             #
#*************************************************************#
def PickRandomGame(message):
    list_ = message.split('game')[1].strip().rstrip().split(',')   # rough-form of games list
    games = [game.strip() for game in list_]                       # whitespace-stripped games list
    if len(games[-1]) < 2: games.pop()                             # removes last item if it's only 1 char
    return choice(games)




#*********************************************************#
#                                                         #
#   Frequency data normalization function                 #
#     Input:   raw timeseries data                        #
#     Outputs: data normalized to minimum of 1,           #
#              year in which the data is normalized to,   #
#              year where the global maximum is reached   #
#                                                         #
#*********************************************************#
def normalize(timeseries, start):
    nozeros = [x for x in timeseries if x != 0]                 # remove zeros from the timeseries
    smallest = min(nozeros)                                     # find minimum value in the timeseries
    norm = [round(value/smallest,5) for value in timeseries]    # normalize the timeseries to the smallest value
    biggest = max(norm)                                         # find the maximum value in the normalized timeseries
    for i in range(len(norm)):
        if norm[i] == 1:  smallest_year = i+start               # find the year in which the smallest value is reached
        if norm[i] == biggest:  biggest_year = i+start          # find the year in which the biggest value is reached
    return norm, smallest_year, biggest_year




#*********************************************************#
#                                                         #
#   Main n-gram searching function                        #
#     Input:   query string                               #
#     Outputs: data normalized to minimum of 1,           #
#              year in which the data is normalized to,   #
#              year where the global maximum is reached,  #
#              actual search term, start year, end year   #
#                                                         #
#*********************************************************#
def get(query):
    url, search, start, end = getURL(query)                            # get ngram url
    with urlopen(url) as response:
        datajson = json.loads(response.read())[0]                      # read the json data
    try:  timeseries = datajson['timeseries']                          # extract the timeseries
    except Exception as e:  raiseException(e,74,__file__)
    norm, smallest_year, biggest_year = normalize(timeseries, start)   # normalize the timeseries
    return norm, smallest_year, biggest_year, search, start, end




#*********************************************************#
#                                                         #
#   URL getter function                                   #
#     Input:   query string, start year,                  #
#              end year, and smoothing amount             #
#     Outputs: google ngram url string,                   #
#              actual search term, start year, end year   #
#                                                         #
#*********************************************************#
def getURL(query, start=1800, end=2019, smoothing=0, **kwargs):
    if ',' in query:
        split_str = query.split(',')                        # split the query string by commas

        if split_str[0][0] != '.':  return                  # only accept queries beginning with '.'
        else:  search = split_str[0][1:]                    # take everything from '.' to ',' for search string

        if ' ' in search:
            search = search.replace(' ', '+')               # spaces need to be '+'s in google ngram URL

        for i in range(1, len(split_str)):
            if 'start' in split_str[i]:
                if '=' in split_str[i]:                                           # if the start year is specified with '='
                    try:  start = int(split_str[i].split('=')[1])                   # get start year as an int
                    except Exception as e:  raiseException(e,103,__file__)
                elif 'at' in split_str[i]:                                        # if the start year is specified with 'at'
                    try:  start = int(split_str[i].split('at')[1])                  # get start year as an int
                    except Exception as e:  raiseException(e,106,__file__)
                elif 'from' in split_str[i]:                                      # if the start year is specified with 'at'
                    try:  start = int(split_str[i].split('from')[1])                # get start year as an int
                    except Exception as e:  raiseException(e,109,__file__)

            if 'end' in split_str[i]:
                if '=' in split_str[i]:                                           # if the end year is specified with '='
                    try:  end = int(split_str[i].split('=')[1])                     # get end year as an int
                    except Exception as e:  raiseException(e,114,__file__)
                elif 'at' in split_str[i]:                                        # if the end year is specified with 'at'
                    try:  end = int(split_str[i].split('at')[1])                    # get end year as an int
                    except Exception as e:  raiseException(e,117,__file__)
                elif 'from' in split_str[i]:                                      # if the end year is specified with 'at'
                    try:  end = int(split_str[i].split('from')[1])                  # get end year as an int
                    except Exception as e:  raiseException(e,120,__file__)

            if 'smoothing' in split_str[i]:
                try:  smoothing = int(split_str[i].split('=')[1])                 # get smoothing amount as an int
                except Exception as e:  raiseException(e,124,__file__)
    
    else:
        search = query[1:]                                  # search string is everything after '.' in query string
        if ' ' in search:
            search = search.replace(' ', '+')               # spaces need to be '+'s in google ngram URL
    
    if start > end:  start, end = end, start                # start year cannot be greater than end year
    if end > 2019 or end < 1500:  end = 2019                # end year can only be between 1500 and 2019
    if smoothing < 0:  smoothing = -smoothing               # smoothing amount cannot be a negative number
    if start > 2019 or start < 1500:  start = 1800          # start year can only be between 1500 and 2019
    
    url = f"https://books.google.com/ngrams/json?content={search}&year_start={start}&year_end={end}&corpus=26&smoothing={smoothing}&case_insensitive=true"
    return url, search, start, end




#*********************************************************#
#                                                         #
#   Step-size calculator                                  #
#     Inputs:  start year, end year                       #
#     Output:  an integer step-size,                      #
#              such that  7 < x-axis tick count < 13      #
#                                                         #
#*********************************************************#
def stepsize(start, end):
    if   end-start < 30:   return 3
    elif end-start > 29    and end-start <=  59:  return 5
    elif end-start > 59    and end-start <= 104:  return 10
    elif end-start > 104   and end-start <= 139:  return 15
    elif end-start > 139   and end-start <  175:  return 20
    elif end-start >= 175  and end-start <  324:  return 25
    elif end-start >= 324  and end-start <  389:  return 30
    elif end-start >= 389  and end-start <  454:  return 35
    elif end-start >= 454: return 40




#*********************************************************#
#                                                         #
#   Plot generating function                              #
#     Input:  search string (passed to get function)      #
#     Output: png image file (saves locally for main)     #
#                                                         #
#*********************************************************#
def plot(query):
    timeseries, smallest_year, biggest_year, search, start, end = get(query)
    print(f"\n\n[plot function]>>>  Searching for {search} from {start} to {end}", end='')
    print(f" with smallest_year = {smallest_year}, and bigggest_year = {biggest_year}\n\n")

    step = stepsize(start,end)
    pyplot.style.use('ggplot')
    pyplot.rcParams['figure.figsize'] = (17, 10)
    pyplot.tight_layout()
    pyplot.subplots_adjust(right=0.93, bottom=0.15)

    pyplot.title(f'"{search.replace("+", " ")}"', font=Path('font/JetBrainsMonoExtraBold.ttf'), fontsize = 35, pad = 30, color='white')
    pyplot.xlabel('year', font=Path('font/JetBrainsMonoMediumItalic.ttf'), fontsize=33, labelpad=41, color='white')
    pyplot.ylabel('frequency', font=Path('font/JetBrainsMonoMediumItalic.ttf'), fontsize=30, labelpad=38, color='white')
    pyplot.xticks(range(start, end, step), font=Path('font/InconsolataRegular.ttf'), fontsize=30, color='white')
    pyplot.yticks(font=Path('font/InconsolataRegular.ttf'), fontsize=30, color='white')

    x = [i+start for i in range(len(timeseries))]
    pyplot.plot(x,timeseries)
    pyplot.axvline(x=biggest_year, color='#BBBBBB', linestyle='--')
    pyplot.axvline(x=smallest_year, color='#BBBBBB', linestyle='--')
    pyplot.savefig('pic.png', format='png', facecolor='#36393f')
    pyplot.close()
