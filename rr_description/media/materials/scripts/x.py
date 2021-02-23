for x in range(60):
    f = open("tag36_11_%05d.material" % x, "a")
    f.write(
"""material tag36_11_%05d
{
    technique
    {
        pass
        {
            texture_unit
            {
                texture tag36_11_%05d.png
                filtering none none none
                scale 1 1
            }
        }
    }
}
    """ % (x, x))
    f.close()
