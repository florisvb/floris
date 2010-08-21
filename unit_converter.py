# unit converter
# for better performance: should stay in imperial if end result is to be imperial: convert all imperial to inches

def units(num, fromunit, tounit):
    # initialize standard units
    num_in_inches = None
    num_in_meters = None
    
    # inter-system unit conversion values
    in2m = 0.0254
    
    # imperial conversion values
    ft2in = 12.
    yd2in = 36.
    mil2in = 0.001
    
    # metric conversion values
    mm2m = 10.**-3
    cm2m = 10.**-2
    dm2m = 10.**-1
    km2m = 10.**3
    
    # imperial name lists
    inches = ['in', 'inch', 'inches']
    feet = ['ft', 'foot', 'feet']
    yards = ['yd', 'yds', 'yard', 'yards']
    mils = ['mil', 'mils']
    
    # metric name lists
    meters = ['m', 'meters', 'meter']
    cmeters = ['cm', 'centimeters', 'centimeter', 'cmeters']
    mmeters = ['mm', 'millimeters', 'millimeter', 'mmeters']
    dmeters = ['dm', 'decimeters', 'decimeter', 'dmeters']
    kmeters = ['km', 'kilometers', 'kilometer', 'kmeters']
    
    ## first convert to meters or inches ##
    # imperial
    if fromunit in feet:
        num_in_inches = num*ft2in
    if fromunit in mils:
        num_in_inches = num*mil2in
    # metric
    if fromunit in mmeters:
        num_in_meters = num*10**-3
    if fromunit in cmeters:
        num_in_meters = num*10**-2
    if fromunit in dmeters:
        num_in_meters = num*10**-1
        
    if fromunit in kmeters:
        num_in_meters = num*10**3
        
    ## calculate all base values ##
    if num_in_meters is None and num_in_inches is None:
        ValueError 'improper units specified'
    if num_in_meters is None:
        num_in_meters = num_in_inches*in2m
    if num_in_inches is None:
        num_in_inches = num_in_meters/in2m
        
    ## output ##
    # metric
    if tounit in meters:        
        return num_in_meters
    if tounit in cmeters:
        return num_in_meters/cm2m
    if tounit in mmeters:
        return num_in_meters/mm2m
    if tounit in dmeters:
        return num_in_meters/dm2m
    if tounit in kmeters:
        return num_in_meters/km2m
    
    # imperial
    if tounit in inches:
        return num_in_inches
    if tounit in feet:
        return num_in_inches/ft2in
    if tounit in mils:
        return num_in_inches/mil2in
    if tounit in yards:
        return num_in_inches/yd2in 
    
    
## some common conversions as short functions

def in2m(num):
    conv = units(num, 'in', 'm')
    return conv
def mil2m(num):
    conv = units(num, 'mil', 'm')
    return conv
def mm2m(num):
    conv = units(num, 'mm', 'm')
    return conv
def cm2m(num):
    conv = units(num, 'cm', 'm')
    return conv
    
    
    
    
    
    
    
    
    
    
    
    
        
