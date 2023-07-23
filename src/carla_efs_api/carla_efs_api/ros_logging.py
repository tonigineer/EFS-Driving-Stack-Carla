from rclpy import logging


def logdebug(msg):
    logging.get_logger("default").debug(msg)


def loginfo(msg):
    logging.get_logger("default").info(msg)


def logwarn(msg):
    logging.get_logger("default").warn(msg)


def logerr(msg):
    logging.get_logger("default").error(msg)


def logfatal(msg):
    logging.get_logger("default").fatal(msg)
