k_consumers = []


def register_k_consumer(consumer):
    k_consumers.append(consumer)


def update_k(k, device):
    for consumer in k_consumers:
        consumer(k, device)
