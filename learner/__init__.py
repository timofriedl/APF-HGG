from .ghgg import GHGGLearner
from .hgg import HGGLearner
from .normal import NormalLearner

learner_collection = {
    'normal': NormalLearner,
    'hgg': HGGLearner,
    'ghgg': GHGGLearner,
}


def create_learner(args):
    return learner_collection[args.learn](args)
