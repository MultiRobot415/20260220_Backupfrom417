import pygame

def init():
    pygame.init()
    win = pygame.display.set_mode((400, 400))
    pygame.display.set_caption("Tello Control - キーボード入力ウィンドウ")
    win.fill((200, 200, 200))
    font = pygame.font.Font(None, 24)
    text1 = font.render("このウィンドウにフォーカスを当ててください", True, (0, 0, 0))
    text2 = font.render("Q: 離陸, E: 着陸", True, (0, 0, 0))
    text3 = font.render("W/S: 上昇/下降", True, (0, 0, 0))
    text4 = font.render("A/D: 左右回転", True, (0, 0, 0))
    text5 = font.render("↑/↓: 前進/後退, ←/→: 左右移動", True, (0, 0, 0))
    text6 = font.render("ESC: 緊急停止", True, (0, 0, 0))
    win.blit(text1, (50, 50))
    win.blit(text2, (50, 100))
    win.blit(text3, (50, 150))
    win.blit(text4, (50, 200))
    win.blit(text5, (50, 250))
    win.blit(text6, (50, 300))
    pygame.display.update()

def getKey(keyName):
    ans = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
    keyInput = pygame.key.get_pressed()
    try:
        if len(keyName) == 1 and keyName.islower():
            myKey = getattr(pygame, f'K_{keyName.lower()}')
        else:
            myKey = getattr(pygame, f'K_{keyName.upper()}')
        if keyInput[myKey]:
            ans = True
            win = pygame.display.get_surface()
            win.fill((200, 200, 200))
            font = pygame.font.Font(None, 24)
            text1 = font.render("このウィンドウにフォーカスを当ててください", True, (0, 0, 0))
            text2 = font.render("Q: 離陸, E: 着陸", True, (0, 0, 0))
            text3 = font.render("W/S: 上昇/下降", True, (0, 0, 0))
            text4 = font.render("A/D: 左右回転", True, (0, 0, 0))
            text5 = font.render("↑/↓: 前進/後退, ←/→: 左右移動", True, (0, 0, 0))
            text6 = font.render("ESC: 緊急停止", True, (0, 0, 0))
            text7 = font.render(f"キー入力: {keyName}", True, (255, 0, 0))
            win.blit(text1, (50, 50))
            win.blit(text2, (50, 100))
            win.blit(text3, (50, 150))
            win.blit(text4, (50, 200))
            win.blit(text5, (50, 250))
            win.blit(text6, (50, 300))
            win.blit(text7, (50, 350))
    except:
        pass
    pygame.display.update()
    return ans

def main():
    if getKey("LEFT"):
        print("Left key pressed")
    if getKey("RIGHT"):
        print("Right key pressed")
    if getKey("UP"):
        print("Up key pressed")
    if getKey("DOWN"):
        print("Down key pressed")
    if getKey("w"):
        print("w key pressed")
    if getKey("a"):
        print("a key pressed")
    if getKey("s"):
        print("s key pressed")
    if getKey("d"):
        print("d key pressed")
    if getKey("q"):
        print("q key pressed")
    if getKey("e"):
        print("e key pressed")

if __name__ == "__main__":
    init()
    while True:
        main()
