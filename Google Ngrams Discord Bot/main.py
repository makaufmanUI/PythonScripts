#####
#
#     Hanldes queries to google ngrams, returns a timeseries of the usage
#       frequency in the google books corpus from 1800-2019 as a line chart
#
#####


import time
import discord
from datetime import datetime as dt
from functions import plot, raiseException, PickRandomGame

TOKEN = 'xxxxx'

client = discord.Client()



@client.event
async def on_ready():
    print(f"\n\n\n>>> Logged in as {client.user.name}, with Client ID [{client.user.id}]")
    print(f"\n>>> Date:  {dt.now().strftime('%m-%d-%Y')},  Time:  {dt.now().strftime('%H:%M:%S')}")
    print("\n>>> Current servers:  ", end='')
    for guild in client.guilds:
        print(guild.name, end='   ')
    print("\n\n\n\n>>> Listening...\n\n")
    await client.change_presence(activity=discord.Game(name='Listening for Queries'))




@client.event
async def on_message(message):
    # DM handler
    if not message.guild and message.author != client.user:
        await message.channel.send("u r gay")
        return

    # Don't listen to self
    if message.author == client.user:
        return
    
    # Start timer
    start = time.perf_counter()



    #****************************#
    #                            #
    #    Search query handler    #
    #                            #
    #****************************#
    if message.content.startswith('.'):
        if "game" in message.content:
            try:
                game = PickRandomGame(message.content)
                await message.channel.send(f"You shall play {game}.")
            except:
                await message.channel.send("IDK. Pick one yourself.")
        else:
            query = message.content
            print(f"\n>>>  {message.author.name} sent:  '{query}'", end='')
            try:  plot(query)
            except Exception as e:  raiseException(e,61,__file__)
            await message.channel.send(file=discord.File('pic.png'))
            print(f"\t--> Completed query in {round(time.perf_counter()-start,3)} seconds\n")
    


    #********************************************************#
    #                                                        #
    #   Search history deletion handler (specified amount)   #
    #                                                        #
    #********************************************************#
    if message.content.startswith('!delete') and 'all' not in message.content:
        requestor = message.author
        if len(message.content) < 8:
            to_delete = 2
        else:
            to_delete_str = message.content[8:]
            try:  to_delete = int(to_delete_str)*2
            except:  to_delete = 2
        if to_delete == 2:
            print(f"\n>>>  {requestor.name} requested deletion of 1 item ({to_delete} messages).")
        else:
            print(f"\n>>>  {requestor.name} requested deletion of {int(to_delete/2)} items ({to_delete} messages).")

        if to_delete > 6:
            await message.channel.send(f"{requestor.name}, you can only delete 3 search histories at a time. To delete more, specify 'all'.")
            print(f">>>  {requestor.name}'s delete request was denied due to too large a size ({int(to_delete/2)} items/{to_delete} messages).")
            
        else:
            count = 0
            requested = to_delete
            async for message in message.channel.history(limit=None):
                count += 1
                
                if message.author == requestor and message.content.startswith('!delete'):
                    count -= 1
                    await message.delete()
                    print(f"-->  Deleted  '{message.content}'  request message.")
                elif message.author == client.user and "you can only" in message.content:
                    count -= 1
                    await message.delete()
                    print(f"-->  Deleted a 'to_delete > 6' warning message.")
                elif message.author == requestor and message.content.startswith('.'):
                    await message.delete()
                    print(f"-->  Deleted message #{count} of {requested} (query '{message.content}').")
                    to_delete -= 1
                    if to_delete == 0:  break
                elif message.author == client.user:
                    await message.delete()
                    print(f"-->  Deleted message #{count} of {requested} (ngram plot).")
                    to_delete -= 1
                    if to_delete == 0:  break

            print(f"\n>>>  Finished deletion request from {requestor.name} in {round(time.perf_counter()-start,3)} seconds.\n")
    


    #*******************************************#
    #                                           #
    #   Search history deletion handler (all)   #
    #                                           #
    #*******************************************#
    if message.content.startswith('!delete') and 'all' in message.content:
        requestor = message.author
        print(f"\n>>>  {requestor.name} requested deletion of all messages.")

        count = 0
        async for message in message.channel.history(limit=None):
            count += 1
            if message.author == requestor and message.content.startswith('!delete'):
                count -= 1
                await message.delete()
                print(f"-->  Deleted  '{message.content}'  request message.")
            elif message.author == client.user and "you can only" in message.content:
                count -= 1
                await message.delete()
                print(f"-->  Deleted a 'to_delete > 6' warning message.")
            elif message.author == requestor and message.content.startswith('.'):
                await message.delete()
                print(f"-->  Deleted message #{count} (query '{message.content}').")
            elif message.author == client.user:
                await message.delete()
                print(f"-->  Deleted message #{count} (ngram plot).")

        print(f"\n>>>  Finished 'all' deletion request from {requestor.name} in {round(time.perf_counter()-start,3)} seconds.\n")




client.run(TOKEN)
