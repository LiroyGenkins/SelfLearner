# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


def check_prior(a):
    if a == 0:
        return 0
    if a == 1:
        return 1
    if a == 2:
        return 2


# и для дистанции на каждом
def check_dist(a):
    if a < 30:
        return 0
    if (a > 30) and (a < 70):
        return 1
    if (a >= 70) and (a < 100):
        return 2
    if a >= 100:
        return 3


def select_side(l, ml, f, mr, r, prior_sector):
    if check_dist(f) == 2:
        return prior_sector
    else:
        offset = check_dist(l)+check_dist(ml)-check_dist(r)-check_dist(mr)
    if offset <= 0:
        return 1
    else:
        return 2


def output_turn(solution):
   if solution == 1:
       return "BIG LEFT"
   if solution == 2:
       return "LEFT"
   if solution == 3:
       return "SMALL LEFT"
   if solution == 4:
       return "SMALL RIGHT"
   if solution == 5:
       return "RIGHT"
   if solution == 6:
       return "BIG RIGHT"
   else:
       return "You have PROBLEM"



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    solution_arr='44434333522261114443444355' \
                 '22661154434443555266614444' \
                 '44445556666633333333222211' \
                 '11333333332222611133333333' \
                 '522261114443443355226661'
    target_sector = 1
    mid = 20
    left = 40
    right = 100
    indl = check_dist(left)
    indm = check_dist(mid)
    indr = check_dist(right)
    indp = check_prior(target_sector)
    print(indp,indl, indm, indr)
    solution_num= 64*indp+16*indl+4*indm+indr
    solution = int(solution_arr[solution_num])
    print("Ситуация номер ",solution_num)
    print('Решение - ',solution)
    print(output_turn(solution))
    # и достаем из массива решение по номеру


# See PyCharm help at https://www.jetbrains.com/help/pycharm/
