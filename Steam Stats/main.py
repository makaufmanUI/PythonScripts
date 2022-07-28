import requests
from matplotlib import pyplot
from stuff import view, clear, dash
from bs4 import BeautifulSoup as bs


def commas(number):
    """
    Converts a number to a string with commas.
    """
    return "{:,}".format(number)

def get_soup(url):
    """
    Get the soup object from the url
    """
    r = requests.get(url)
    return bs(r.text, 'html.parser')

def get_total_players(soup):
    """
    Get the total number of players
    """
    content = soup.find('table', {'class': 'users_count_table'}).text.split('\n')
    current_players = int(content[9].replace(',', ''))
    peak_players = int(content[10].replace(',', ''))
    return current_players, peak_players

def get_list(soup):
    """
    Get the entire 300 long list of current players, peak today, and game
    """
    table = soup.find('div', {'id': 'detailStats'})
    tableList = table.text.replace(',', '').split('\n')
    newtableList = []
    newtableListCommas = []
    for thing in tableList:
        if thing == '' or thing == '\xa0' or thing == 'Current Players' or thing == 'Peak Today' or thing == 'Game':
            pass
        else:
            if 'View all' in thing:
                pass
            else:
                try:
                    thing = int(thing)
                    newtableList.append(thing)
                    newtableListCommas.append(commas(thing))
                except:
                    newtableList.append(thing)
                    newtableListCommas.append(thing)
    return newtableList, newtableListCommas

# @jit
def get_lists(List, ListCommas=None):
    """
    Turns the 300 long list of everything into 3 separate lists
    """
    current_players = []
    peak_players = []
    game = []
    for i in range(0, len(List), 3):
        current_players.append(int(List[i]))
    for i in range(1, len(List), 3):
        peak_players.append(int(List[i]))
    
    n = 1
    for i in range(2, len(List), 3):
        game.append(f'[{n}] '+List[i])
        n += 1
    if ListCommas != None:
        current_players_commas = []
        peak_players_commas = []
        for i in range(len(current_players)):
            current_players_commas.append(commas(current_players[i]))
        for i in range(len(peak_players)):
            peak_players_commas.append(commas(peak_players[i]))
        return current_players, peak_players, game, current_players_commas, peak_players_commas
    return current_players, peak_players, game


def dictionary(current_players, peak_players, game, current_players_commas=None, peak_players_commas=None):
    """
    Turns the 3 lists into a dictionary
    """
    Dict = {}
    for i in range(len(current_players)):
        Dict[game[i]] = {'Peak players': peak_players[i], 'Current Players': current_players[i]}
    if current_players_commas != None and peak_players_commas != None:
        Dict_commas = {}
        for i in range(len(current_players_commas)):
            Dict_commas[game[i]] = {'Peak players': peak_players_commas[i], 'Current Players': current_players_commas[i]}
        return Dict, Dict_commas
    return Dict

def plot(Dict):
    """
    Plots the data
    """
    pyplot.tight_layout()
    pyplot.style.use('ggplot')
    pyplot.subplots_adjust(right=0.93, bottom=0.15)
    i = 0
    x, y = [], []
    for key in Dict:
        y.append(Dict[key]['Current Players'])
        x.append(i)
        i += 1
    pyplot.step(x, y)
    pyplot.xlabel('Games, by number of players')
    pyplot.ylabel('Current Player Count')
    pyplot.title('Current Player Count by Game')
    pyplot.show()

def search(game, games):
    try: game = int(game)
    except: pass
    if isinstance(game, int):
        if game < 1 or game > 100:
            return "NONE", "NONE"
        else:
            for each_game in games:
                if game == int(each_game.split('[')[1].split(']')[0]):
                    return each_game, data_commas[each_game]
    elif '-' in game and len(game) < 7:
        start = int(game.split('-')[0])
        end = int(game.split('-')[1])
        if start >= end:
            print('\n\n* Invalid range *\n\n')
            return "NONE", "NONE"
        keys = list(data_commas.keys())
        print('\n\n')
        print(f"\n\n\nSEARCH RESULT:\n{dash(14)}")
        for i in range(start-1, end, 1):
            this_game = keys[i]
            print(f"\n\n{this_game}:\n> Peak players: {data_commas[this_game]['Peak players']}\n> Current players: {data_commas[this_game]['Current Players']}\n\n\n")
    else:
        for each_game in games:
            if game.lower() in each_game.split('] ')[1].lower():
                return each_game, data_commas[each_game]
    return "NONE", "NONE"



if __name__ == '__main__':
    clear()
    global data
    global data_commas

    all = get_soup('https://store.steampowered.com/stats/')
    current_total, peak_total = get_total_players(all)
    print(f"TOP 100 TOTAL:\n{dash(14)}\n")
    print(f"> Peak Players: {commas(peak_total)}")
    print(f"> Current Players: {commas(current_total)}")
    
    tableList, tableList_commas = get_list(all)

    current_players, peak_players, games, current_players_commas, peak_players_commas = get_lists(tableList, tableList_commas)
    data, data_commas = dictionary(current_players, peak_players, games, current_players_commas, peak_players_commas)

    
    search_ = ''
    print('\n\n')
    print('* Search by name, by integer rank in player count (1-100), or by range (ex: 1-10) *')
    print('\n\n')
    while search_.lower() != 'q':
        search_ = input('>> Search for a game (q to quit):  ')
        if search_ != 'q':
            match, game_data = search(search_, games)
            if game_data != 'NONE':
                print(f"\n\n\nSEARCH RESULT:\n{dash(14)}\n\n{match}:\n> Peak players: {game_data['Peak players']}\n> Current players: {game_data['Current Players']}\n\n\n")
        else: print('\n\n\n')


    view(data_commas)
    plot(data)