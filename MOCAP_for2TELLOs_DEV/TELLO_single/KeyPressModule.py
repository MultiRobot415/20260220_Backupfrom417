import pygame

def init():
    pygame.init()
    win = pygame.display.set_mode((400, 400))
    pygame.display.set_caption("Keyboard Input Window")

def getKey(keyName):
    ans = False
    for eve in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    
    # 小文字のキーを処理するための修正
    if len(keyName) == 1 and keyName.islower():
        myKey = getattr(pygame, 'K_{}'.format(keyName))
    else:
        myKey = getattr(pygame, 'K_{}'.format(keyName.upper()))
        
    if keyInput[myKey]:
        ans = True
    pygame.display.update()
    return ans

def main():
    if getKey("LEFT"):
        print("Left key pressed")
    if getKey("RIGHT"):
        print("Right key pressed")
    if getKey("w"):
        print("w key pressed")
    if getKey("a"):
        print("a key pressed")
    if getKey("s"):
        print("s key pressed")
    if getKey("d"):
        print("d key pressed")

if __name__ == "__main__":
    init()
    while True:
        main()
