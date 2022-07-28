import json
import time
import yagmail
from urllib.request import urlopen

YAG = yagmail.SMTP("scrubberbot@gmail.com", "xxxxx")



def getfrom(url):
    """
    Gets the JSON data from the specified URL.
    """
    try:                                            # Try to get the JSON data (HTTP timeout possible)
        with urlopen(url) as response:
            data = response.read().decode('utf-8')       # a `bytes` object, need to decode to `str`
            jsondata = json.loads(data)                  # a `str` object, can be used as a JSON object
    except Exception as e:
        raiseException(e)                           # If an error occurs, raise the exception
        jsondata = -1                               # Return -1 to indicate the error
        
    return jsondata



def parse(data):
    """
    Parses a `dict`/JSON object for the relevant information.
    """
    id = data['data']['children'][0]['data']['id']                        # ID of post
    link = data['data']['children'][0]['data']['url']                     # Link to product
    Time = data['data']['children'][0]['data']['created_utc']             # UTC time in seconds (Unix time)
    title = data['data']['children'][0]['data']['title'].split(']')       # Title of post
    component = data['data']['children'][0]['data']['link_flair_text']    # Component type

    try: title = title[1]                           # Try to remove the [ ... ] from the title
    except Exception as e:
        raiseException(e)                           # If an error occurs, raise the exception
        title = title[0]                            # Just use the first part of the title

    if title[0] == ' ': title = title[1:]                                 # Remove the leading space following the closing brace ] if it exists

    if '&amp;' in link: link = link.split('&amp;')[0]                     # Remove &amp; from link if present
    if '/ref=' in link: link = link.split('/ref=')[0]                     # Remove /ref= from link if present
    Time = time.strftime('%m-%d  %H:%M:%S', time.localtime(Time))         # Convert time to local time in MM-DD HH:MM:SS format

    return id, [Time, component, title, link]



def Print(details):
    """
    Prints the details of the post.
    """
    print(f"\n\n\n   [ {details[0]} ]  --  New {details[1]} post found.")
    print('   ', end='')
    for i in range(len(details[0])+len(details[1])+26): print('~',end='')     # Print a ~~~~ line under new post text

    print(f"\n\n\t--> Title:  {details[2]}")
    print(f"\n\t --> Link:  {details[3]}\n")



def email(receiver, details):
    """
    Sends the details of the post to the specified email(s).
    """
    subject = f"A {details[1]} has been found on r/buildapcsales!"
    body = f"-Title of post: \n{details[2]}\n\n\n-Link to product: \n{details[3]}\n"
    YAG.send (
        to = receiver,
        subject = subject,
        contents = body
    )



def raiseException(e):
    """
    Raises an exception (prints to console).
    """
    Time = time.strftime('%m-%d  %H:%M:%S')
    print(f"\n\n   [ {Time} ]  --  An exception has occurred.")
    print('   ', end='')
    for i in range(len(Time)+36): print('~',end='')     # Print a ~~~~ line under exception text
    print(f"\n\n\t-->  {e}\n\n")
