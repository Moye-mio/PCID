import cv2 as cv

ModelId = ["1", "2", "3", "4"]
FileName = ["ai.jpg", "dgi.png", "gt.png", "hole.png", "ours.png", "telea.png"]

for id in ModelId:
    image = cv.imread("jet/"+id+"/mask.png", 0)

    minx = image.shape[0]
    miny = image.shape[1]
    maxx = 0
    maxy = 0

    for x in range(image.shape[0]):
        for y in range(image.shape[1]):
            if image[x, y] == 255:
                minx = min(minx, x)
                miny = min(miny, y)
                maxx = max(maxx, x)
                maxy = max(maxy, y)
                
    print(minx, maxx)
    print(miny, maxy)

    offset = 30

    avex = int((minx + maxx) / 2)
    avey = int((miny + maxy) / 2)
    span = int(max(maxx - minx, maxy - miny) / 2) + offset

    avex = span if avex < span else avex
    avex = image.shape[0] - span if avex + span > image.shape[0] else avex
    avey = span if avey < span else avey
    avey = image.shape[1] - span if avey + span > image.shape[1] else avey

    
    for name in FileName:
            raw = cv.imread("jet/"+id+"/"+name)
            cut = raw[avex - span : avex + span, avey - span : avey + span]
            cv.imwrite("jet/"+id+"/cut/"+name, cut)


