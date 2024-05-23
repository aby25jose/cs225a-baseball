import redis

r = redis.Redis(
    host='localhost',
    port=6379,
    decode_responses=True
)

radio = r.pubsub()
radio.subscribe("button")
messages = radio.listen()
for msg in messages:
    print(msg["data"])