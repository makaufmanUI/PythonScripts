from time import sleep
from functions import parse, getfrom, Print, email

WATCHLIST = ['GPU', 'CPU']
RECEIVER = "mkaufmanv2@gmail.com"
URL = 'https://api.reddit.com/r/buildapcsales/new?limit=1'




if __name__ == '__main__':

    lastID = ''

    while True:
        jsondata = getfrom(URL)

        if jsondata != -1:
            postID, details = parse(jsondata)

            if postID != lastID:
                lastID = postID
                Print(details)

                if details[1] in WATCHLIST:
                    email(RECEIVER, details)
        
        sleep(30)
