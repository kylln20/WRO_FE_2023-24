import pygame
import random
import sys
pygame.init()
pygame.font.init()
my_font = pygame.font.SysFont('Comic Sans MS', 30)

width = 500
# Set up display
screen = pygame.display.set_mode((width, width))
pygame.display.set_caption('WRO Obstacle Challenge Randomizer')

rect = pygame.Rect(10, 10, width - 20, width - 20)
rect2 = pygame.Rect(0, 0, 160, 160)
rect3 = pygame.Rect(0, 0, 140, 140)

combinations = [
    ["G", "", "", "", "", ""], 
    ["R", "", "", "", "", ""], 
    ["", "", "G", "", "", ""], 
    ["", "", "R", "", "", ""], 
    ["", "", "", "", "G", ""], 
    ["", "", "", "", "R", ""], 
    ["", "G", "", "", "", ""],
    ["", "R", "", "", "", ""],  
    ["", "", "", "G", "", ""], 
    ["", "", "", "R", "", ""], 
    ["", "", "", "", "G", ""], 
    ["", "", "", "", "R", ""], 
    ["", "G", "", "", "G", ""], 
    ["", "G", "", "", "R", ""], 
    ["", "R", "", "", "G", ""], 
    ["", "G", "", "", "R", ""], 
    ["", "R", "", "", "G", ""], 
    ["", "R", "", "", "R", ""], 
    ["G", "", "", "", "", "G"], 
    ["G", "", "", "", "", "R"], 
    ["R", "", "", "", "", "G"], 
    ["G", "", "", "", "", "R"], 
    ["R", "", "", "", "", "G"], 
    ["R", "", "", "", "", "R"], 
    ["G", "", "", "", "G", ""], 
    ["G", "", "", "", "R", ""], 
    ["R", "", "", "", "G", ""], 
    ["G", "", "", "", "R", ""], 
    ["R", "", "", "", "G", ""], 
    ["R", "", "", "", "R", ""], 
    ["", "G", "", "", "", "G"], 
    ["", "G", "", "", "", "R"], 
    ["", "R", "", "", "", "G"], 
    ["", "G", "", "", "", "R"], 
    ["", "R", "", "", "", "G"], 
    ["", "R", "", "", "", "R"], 
]

pLotHeight = 40
pLotWidth = 5
pWidth = 15




lotPos = random.randint(1, 4)
turnDir = random.randint(1, 2)

text = "CCW" if turnDir == 1 else "CW"

start = random.randint(1, 4)



print(lotPos)
if lotPos % 2 == 0: 
    lLimit = pygame.Rect(0, 0, pLotHeight, pLotWidth)
    rLimit = pygame.Rect(0, 0, pLotHeight, pLotWidth)
    
else: 
    lLimit = pygame.Rect(0, 0, pLotWidth, pLotHeight)
    rLimit = pygame.Rect(0, 0, pLotWidth, pLotHeight)

x = 170
y = 30

if lotPos == 1: #top
    lLimit.center = x + 30, y
    rLimit.center = x, y
elif lotPos == 2: #left 
    lLimit.center = y, x + 130
    rLimit.center = y, x + 160
elif lotPos == 3: #bottom
    lLimit.center = x + 130, 470
    rLimit.center = x + 160, 470
elif lotPos == 4: #right
    lLimit.center = 470, x + 30,
    rLimit.center = 470, x

car = pygame.Rect(0, 0, pWidth, pWidth)

if start == 1: #top
    car.center = x + 80, 90
elif start == 2: #left 
    car.center = 90, x + 80
elif start == 3: #bottom
    car.center = x + 80, 410
elif start == 4: #right
    car.center = 410, x + 80



rc = []
for i in range(4):
    
    while True: 
        r = random.randint(0, 35)

        if i + 1 == lotPos: 
            if combinations[r][1] == combinations[r][3] and combinations[r][3] == combinations[r][5]: 
                if r not in rc: 
                    rc.append(r)
                    print(r)
                    break
        else: 
            if r not in rc: 
                rc.append(r)
                break


        

class Section(): 
    def __init__(self, number): 
        self.pillars = combinations[rc[number-1]]
        self.number = number
        self.pRects = [pygame.Rect(0, 0, pWidth, pWidth), pygame.Rect(0, 0, pWidth, pWidth), pygame.Rect(0, 0, pWidth, pWidth), pygame.Rect(0, 0, pWidth, pWidth), pygame.Rect(0, 0, pWidth, pWidth), pygame.Rect(0, 0, pWidth, pWidth)]
        #lt, lb, mt, mb, rt, rb

        if self.number == 1: #top section
            t = 70
            b = 110
            x = 170
            self.pRects[0].center = x + 160, b
            self.pRects[1].center = x + 160, t
            self.pRects[2].center = x + 80, b
            self.pRects[3].center = x + 80, t
            self.pRects[4].center = x, b
            self.pRects[5].center = x, t
        elif self.number == 2: #left section
            t = 70
            b = 110
            x = 170
            self.pRects[0].center = b, x
            self.pRects[1].center = t, x
            self.pRects[2].center = b, x + 80
            self.pRects[3].center = t, x + 80
            self.pRects[4].center = b, x + 160
            self.pRects[5].center = t, x + 160
        
        elif self.number == 3: #bottom section
            t = 390
            b = 430
            x = 170
            self.pRects[0].center = x, t
            self.pRects[1].center = x, b
            self.pRects[2].center = x + 80, t
            self.pRects[3].center = x + 80, b
            self.pRects[4].center = x + 80*2, t
            self.pRects[5].center = x + 80*2, b
        
        elif self.number == 4: #right section
            t = 390
            b = 430
            x = 170
            self.pRects[0].center = t, x + 160
            self.pRects[1].center = b, x + 160
            self.pRects[2].center = t, x + 80
            self.pRects[3].center = b, x + 80
            self.pRects[4].center = t, x
            self.pRects[5].center = b, x

rect2.center = width//2, width//2
rect3.center = width//2, width//2
# Main loop
sections = [Section(1), Section(2), Section(3), Section(4)]

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False
                break

    screen.fill((0, 0, 0))  # Fill the screen with white

    pygame.draw.rect(screen, (255, 255, 255), rect)
    pygame.draw.rect(screen, (0, 0, 0), rect2)
    pygame.draw.rect(screen, (255, 255, 255), rect3)

    for section in sections: 
        for i in range(6): 
            if section.pillars[i] == "G": 
                color = (0, 255, 0)
            elif section.pillars[i] == "R": 
                color = (255, 0, 0)
            else: 
                color = (0, 0, 0)
                pygame.draw.rect(screen, color, section.pRects[i], 1)
                continue
            
            pygame.draw.rect(screen, color, section.pRects[i])
    
    pygame.draw.rect(screen, (255, 0, 255), lLimit)
    pygame.draw.rect(screen, (255, 0, 255), rLimit)

    pygame.draw.rect(screen, (0, 0, 0), car)

    text_surface = my_font.render(text, False, (0, 0, 0))
    screen.blit(text_surface, (220, 220))

    pygame.display.flip()

pygame.quit()
sys.exit()
