class Subject:
    def __init__(self):
        self.observer = []

    def attach(self, observer):
        self.observer.append(observer)
        return observer
    
    def detach(self, observer):
        print(f'Subject detach {observer}')

    def notify(self):
        print(f'Subject notify')

class Observer:
    def update(self):
        print(f'Observer update')

class ConcreteSubject(Subject):
    def __int__(self, parameter1, subject_state):
        super().__init__()
        self.subject_state = subject_state
        self.parameter1 = parameter1
        print(f'Concrete Subject {self.subject_state}')

class ConcreteObserver(Observer):
    def __init__(self, parameter1, observer_state):
        super().__init__()
        self.observer_state = observer_state
        self.parameter1 = parameter1
        print(f'Concrete Observer {self.observer_state}')


test = Subject()
# test_message1 = test.attach('Pallavi here')
# test_message2 = test.attach('hi')
# publisher_main = ConcreteObserver(test, 'just checking')
# subscriber_1 = ConcreteSubject(test, 'double check')
# x = test.attach(publisher_main)
# y = test.attach(subscriber_1)

weather_station = ConcreteSubject()
display1 = ConcreteObserver(weather_station, 'display1')
display2 = ConcreteObserver(weather_station, 'display2')
z = weather_station.attach(display1)
print(z)
