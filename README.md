## Tùy chọn khi chạy

### Nhập key 9Router

`--prompt-key` hiện lời nhắc nhập key nếu chưa có biến
`ROUTER_API_KEY`. Key không hiện trên màn hình.

Nếu đã đặt `ROUTER_API_KEY` trong terminal, có thể bỏ
`--prompt-key`.

### Chọn mô hình

Mô hình mặc định: `gemini/gemini-3.5-flash-lite`.
Có thể bỏ `--model` để dùng mặc định.

Để đổi mô hình, truyền đúng tên model được 9Router hỗ trợ:

```bash
ros2 run ur3_llm_control llm_task \
  --prompt-key \
  --model "TEN_MODEL_TRONG_9ROUTER" \
  --command "Cho khối đỏ vào B"
```

Thay `TEN_MODEL_TRONG_9ROUTER` bằng tên lấy từ danh sách
model của router. Key phải có quyền sử dụng model đó.
Có thể thêm `--plan-only` để kiểm tra kế hoạch trước khi
cho robot chuyển động.

### Thay mã sinh viên

MSSV mặc định: `23020744`.
Với mã này, nhiệm vụ là vàng → A, đỏ → B, xanh dương → C.

Đổi MSSV bằng `--student-id`:

```bash
ros2 run ur3_llm_control llm_task \
  --prompt-key \
  --student-id "MSSV_CUA_BAN" \
  --student-name "Ho ten cua ban" \
  --command "Arrange all objects according to my student ID"
```

Thay `MSSV_CUA_BAN` bằng mã sinh viên thực tế. Hệ thống dùng
hai chữ số cuối để chọn thứ tự sắp xếp theo quy tắc của bài.

### Lưu ý vận hành

- URL 9Router mặc định: `http://localhost:20128/v1`.
  Đổi bằng `--base-url "URL_9ROUTER"`.
- Chỉ chạy một phiên mô phỏng và một chương trình điều khiển.
- Sau lượt gắp–đặt, khởi động lại mô phỏng để thử nhiệm vụ mới.
- `--plan-only`: chỉ lập và kiểm tra kế hoạch, không chuyển động.
- `--evidence`: tùy chọn lưu kết quả; không bắt buộc.
- Không lưu key vào mã nguồn hoặc đưa lên Git.
- Bộ mã rút gọn chưa được build/chạy lại độc lập trên máy sạch.
