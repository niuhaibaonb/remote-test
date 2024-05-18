
class TruckDictionary:
    def __init__(self):
        self.off_list = {"v.0": 400, "v.1": 390, "v.2": 390, "v.3": 360, "v.4": 290, "v.5": 250, "v.6": 220, "v.7": 200}
        self.CAT_i = {"v.8": 300}

    def insert_and_sort(self):
        # 将CAT_i字典插入到off_list字典中
        self.off_list.update(self.CAT_i)

        sorted_items = sorted(self.off_list.items(), key=lambda item: item[1])

        sorted_list = [list(item) for item in sorted_items]

        # 找到key="v.8"的次序值
        v8_index = sorted_list.index([self.CAT_i['v.8'], 'v.8']) // 2

        # 输出排序后的次序值
        return v8_index


