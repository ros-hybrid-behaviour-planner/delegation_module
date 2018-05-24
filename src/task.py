

class Task(object):

    def __init__(self, auction_id, auctioneer_name):
        self.__auction_id = auction_id
        self.__auctioneer_name = auctioneer_name

    def get_auction_id(self):
        return self.__auction_id

    def get_auctioneer_name(self):
        return self.__auctioneer_name