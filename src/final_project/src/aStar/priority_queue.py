class PriorityQueue():
    def __init__(self):
        self.queue = []
        self.length = 0

    def is_empty(self):
        return self.length==0

    def push(self,data):
        """Inserts the data object into the self.queue priority queue. Has no return value
        Args:
            data (tuple): An entry of the form (value, priority)
        """
        #data should be a tuple of (val, priority)
        if self.is_empty():
            self.queue.append(data)
            self.length+=1
            return

        inserted = False
        for i in range(len(self.queue)):    #Loop through the queue
            if data[1] < self.queue[i][1]:  #check if the inserting data has lower priority
                self.queue.insert(i, data)  #insert if found
                inserted = True
                self.length+=1
                return
        if inserted == False:
            self.queue.append(data)         #If you make it to the end and nothing inserted, just insert at the end
            self.length+=1
            return

    def pop(self):
        """Removes the item with lowest priority from the queue and returns it.
        """

        val = self.queue[0]
        if self.length ==1: #If the size is one, assume empty after remove
            self.queue = []
        else:
            self.queue = self.queue[1:] #Make the queue the next set of things in queue
        self.length-=1 #decriment the length counter
        return val

if __name__ == "__main__":
    #Test the pq
    pq = PriorityQueue()

    #example data
    test_1 = ("hi", -1)
    test_2 = (12345, 2)
    test_3 = ("data", -10)

    #put on the queue
    pq.push(test_1)
    pq.push(test_2)
    pq.push(test_3)

    #print queue
    print("pq: {}".format(pq.queue))

    #remove from queue
    popped_val = pq.pop()
    print(popped_val)

    print("New pq: {}".format(pq.queue))