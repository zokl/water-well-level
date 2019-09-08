#!/usr/local/bin/lua

require("math")

filename = 'voda.dat'

interval = 10

local open = io.open

local function read_file(path)
    local file = open(path, "rb") -- r read mode and b binary mode
    if not file then return nil end
    local content = file:read "*a" -- *a or *all reads the whole file
    file:close()
    return content
end

local fileContent = read_file('bb.json');
-- print (fileContent);

function lines_from(file)
--   if not file_exists(file) then return {} end
  lines = {}
  for line in io.lines(file) do 
    lines[#lines + 1] = line
  end
  return lines
end


function get_min(data)
    min = split(data[1], " ")[2]
    index = split(data[1], " ")[1]
    for a, b in pairs(data) do
        -- print(">>>>>" .. tostring(b))
        if split(b, " ")[2] <= min then
            min = split(b, " ")[2]
            index = split(b, " ")[1]
        end
    end
    -- print("Nejmensi: " .. tostring(index) .. " " ..tostring(min))
    return {index, min}
end

function split(s, delimiter)
    result = {};
    for match in (s..delimiter):gmatch("(.-)"..delimiter) do
        table.insert(result, match);
    end
    return result;
end

mintmp = {}

data = {}

for index,value in pairs(lines_from(filename)) do

    -- print(split(value, " ")[2])
    table.insert(mintmp, value)

    if index % interval == 0 then
        -- print(" ")
        -- print("10 hodnot")
        min = get_min(mintmp)
        print(min[1], min[2]) 
        -- print(" ")
        mintmp = {}
    end

end

