
  
#Python libraries that we need to import for our bot
import random
from flask import Flask, request
from pymessenger.bot import Bot
import pyowm

owm = pyowm.OWM('571422358bc757f9337694b14d65bd39')  # You MUST provide a valid API key
app = Flask(__name__)
ACCESS_TOKEN = 'EAAFtJIok24wBACjMf7CNXupFMQwoGvpBf1mrAZBEp7cZAVnvDlgpqZBisSjyfhFZAQcy3F2oWmTOjpRKEZA3dfyaCqvkuJYWW4EQGEfXIdcKyox8grcZArTYzr1o3D8dAU0O6MHu42t6bRbskQntlhd5AbOKF62oN38U6Vj7dcjQZDZD'
VERIFY_TOKEN = 'SIMON'
bot = Bot(ACCESS_TOKEN)

#We will receive messages that Facebook sends our bot at this endpoint 
@app.route("/", methods=['GET', 'POST'])
def receive_message():
    #try:
        if request.method == 'GET':
            """Before allowing people to message your bot, Facebook has implemented a verify token
            that confirms all requests that your bot receives came from Facebook.""" 
            token_sent = request.args.get("hub.verify_token")
            return verify_fb_token(token_sent)
        #if the request was not get, it must be POST and we can just proceed with sending a message back to user
        else:
            # get whatever message a user sent the bot
           output = request.get_json()
           for event in output['entry']:
              messaging = event['messaging']
              for message in messaging:
                if message.get('message'):
                    #Facebook Messenger ID for user so we know where to send response back to
                    recipient_id = message['sender']['id']
                    if message['message'].get('text'):
                        input_message = message['message'].get('text')
                        response_sent_text = get_message(input_message)
                        send_message(recipient_id, response_sent_text)
                    #if user sends us a GIF, photo,video, or any other non-text item
                    if message['message'].get('attachments'):
                        input_message == 'null'
                        response_sent_nontext = get_message(input_message)
                        send_message(recipient_id, response_sent_nontext)
    #except:
        #print('Fak')
    #return "Message Processed"
        return "Message Processed"


def verify_fb_token(token_sent):
    #take token sent by facebook and verify it matches the verify token you sent
    #if they match, allow the request, else return an error 
    if token_sent == VERIFY_TOKEN:
        return request.args.get("hub.challenge")
    return 'Invalid verification token'


#chooses a random message to send to the user
def get_message(input_message):
    if input_message == 'Hola':
        response = 'Greetings! Welcome to Moca!'
    elif input_message == 'Hello there':
        response = 'General Kenobi'
    elif input_message == 'roll d20':
        response = str(random.randint(1,20))
    elif input_message == 'roll d100':
        response = str(random.randint(1,100))
    elif 'elp' in input_message:
        response = 'Thanks for using Moca! Type weather for weather information catered for Mexicali, type other commands to see if you can find the easter eggs!'
    elif 'null':
        response = 'Please use text only!'
    elif 'eather' in input_message:
        observation = owm.weather_at_place('Mexicali,MX')
        w = observation.get_weather()
        #sample_responses = ["The weather is nice today!", "UV Index high, please procceed with caution!", "CO levels high, don't stay outside!"]
        #response = random.choice(sample_responses)
        air_quality_index = random.randint(1,100)
        uv_index = random.randint(1,40)
        temperature = w.get_temperature('celsius')
        humidity = w.get_humidity()
        if air_quality_index>20 and air_quality_index<40:
            air_caution = 'Air quality poor, please take precautions and limit outside activity\n'
        elif air_quality_index>40:
            air_caution = 'Air quality severely poor, please avoid outside activity\n'
        else:
            air_caution = ''
        if uv_index>20 and uv_index<40:
            uv_caution = 'UV index moderately high, please use SPF40 sunblock\n'
        elif uv_index>40:
            uv_caution = 'UV index severely high, please avoid outside activity\n'
        else:
            uv_caution = ''
        response = (('UV index: '+str(uv_index)+'\n')+('Temperature: '+str(temperature)+'ºC\n')+('Humidity: '+str(humidity)+'%\n')+('Air quality: '+str(air_quality_index)+'\n')+air_caution+uv_caution)
    else: response = 'Hello! Please enter a command to get a response, type help for a full list of commands'
    # return selected item to the user
    #return random.choice(sample_responses)
    return response
#uses PyMessenger to send response to user
def send_message(recipient_id, response):
    #sends user the text message provided via input response parameter
    bot.send_text_message(recipient_id, response)
    return "success"

if __name__ == "__main__":
    app.run()


