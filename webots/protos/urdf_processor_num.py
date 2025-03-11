import re
import sys

def process_urdf_content(content):
    """处理URDF内容:将小数保留4位,将非常小的数字转换为0"""
    # 正则表达式匹配浮点数（包括科学计数法）
    pattern = re.compile(r'(-?\d+\.\d+(?:E[+-]\d+)?)')
    
    # 处理每个匹配到的数字
    def replace_number(match):
        num_str = match.group(0)
        try:
            num = float(num_str)
            # 如果绝对值非常小，将其转换为0
            if abs(num) < 1e-10:
                return "0"
            # 否则保留4位小数
            return f"{num:.4f}"
        except ValueError:
            return num_str
    
    # 替换内容中的所有匹配数字
    processed_content = pattern.sub(replace_number, content)
    
    return processed_content

def process_urdf_file(input_file, output_file=None):
    """处理URDF文件:将小数保留4位,将非常小的数字转换为0"""
    # 读取输入文件
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 处理内容
    processed_content = process_urdf_content(content)
    
    # 将处理后的内容写入输出文件或返回
    if output_file:
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(processed_content)
        print(f"处理完成，结果已保存到 {output_file}")
    else:
        return processed_content

if __name__ == "__main__":
    # 检查是否提供了命令行参数
    if len(sys.argv) >= 2:
        input_file = sys.argv[1]
        output_file = sys.argv[2] if len(sys.argv) >= 3 else None
        result = process_urdf_file(input_file, output_file)
        if result:
            print(result)
    else:
        # 如果没有提供命令行参数，可以在这里处理文件内容
        # 例如，可以处理预定义的文件名
        input_file = "./boat/boatNo.urdf"
        output_file = "./boat/boat.urdf"
        try:
            process_urdf_file(input_file, output_file)
        except FileNotFoundError:
            print(f"文件 {input_file} 不存在。请确保文件路径正确,或直接粘贴URDF内容到代码中处理。")
            
        # 或者处理粘贴的内容
        """
        urdf_content = ""\"您粘贴的URDF内容""\"
        processed_content = process_urdf_content(urdf_content)
        print(processed_content)
        """