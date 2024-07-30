from .ddpg import DDPG as DDPG1
from .ddpg2 import DDPG as DDPG2
import os


def create_agent(args):
    agent = {
        'ddpg': DDPG1,
        'ddpg2': DDPG2
    }[args.alg](args)

    load_dir = os.path.join(".", "log", "{}-{}-{}".format(args.alg, args.env, args.learn))
    if os.path.exists(load_dir):
        files = os.listdir(load_dir)
        max_epoch = -1
        latest_file = None
        for file in files:
            if file.startswith("saved_policy-") and file.endswith(".index"):
                epoch_str = file[len("saved_policy-"):file.find(".index")]
                if epoch_str != "best":
                    epoch = int(epoch_str)
                    if epoch > max_epoch:
                        max_epoch = epoch
                        latest_file = file

        if latest_file is not None:
            load_path = os.path.join(load_dir, latest_file)
            agent.load(load_path)
            print("Restored epoch {}".format(max_epoch))
            args.start_epoch = max_epoch + 1

    return agent
