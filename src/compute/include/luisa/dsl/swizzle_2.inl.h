[[nodiscard]] auto xx() const noexcept { return dsl::def<Vector<T, 2>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 2>>(), this->expression(), 2u, 0x00u)); }
[[nodiscard]] auto xy() const noexcept { return dsl::def<Vector<T, 2>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 2>>(), this->expression(), 2u, 0x10u)); }
[[nodiscard]] auto yx() const noexcept { return dsl::def<Vector<T, 2>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 2>>(), this->expression(), 2u, 0x01u)); }
[[nodiscard]] auto yy() const noexcept { return dsl::def<Vector<T, 2>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 2>>(), this->expression(), 2u, 0x11u)); }
[[nodiscard]] auto xxx() const noexcept { return dsl::def<Vector<T, 3>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 3>>(), this->expression(), 3u, 0x000u)); }
[[nodiscard]] auto xxy() const noexcept { return dsl::def<Vector<T, 3>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 3>>(), this->expression(), 3u, 0x100u)); }
[[nodiscard]] auto xyx() const noexcept { return dsl::def<Vector<T, 3>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 3>>(), this->expression(), 3u, 0x010u)); }
[[nodiscard]] auto xyy() const noexcept { return dsl::def<Vector<T, 3>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 3>>(), this->expression(), 3u, 0x110u)); }
[[nodiscard]] auto yxx() const noexcept { return dsl::def<Vector<T, 3>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 3>>(), this->expression(), 3u, 0x001u)); }
[[nodiscard]] auto yxy() const noexcept { return dsl::def<Vector<T, 3>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 3>>(), this->expression(), 3u, 0x101u)); }
[[nodiscard]] auto yyx() const noexcept { return dsl::def<Vector<T, 3>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 3>>(), this->expression(), 3u, 0x011u)); }
[[nodiscard]] auto yyy() const noexcept { return dsl::def<Vector<T, 3>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 3>>(), this->expression(), 3u, 0x111u)); }
[[nodiscard]] auto xxxx() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x0000u)); }
[[nodiscard]] auto xxxy() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x1000u)); }
[[nodiscard]] auto xxyx() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x0100u)); }
[[nodiscard]] auto xxyy() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x1100u)); }
[[nodiscard]] auto xyxx() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x0010u)); }
[[nodiscard]] auto xyxy() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x1010u)); }
[[nodiscard]] auto xyyx() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x0110u)); }
[[nodiscard]] auto xyyy() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x1110u)); }
[[nodiscard]] auto yxxx() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x0001u)); }
[[nodiscard]] auto yxxy() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x1001u)); }
[[nodiscard]] auto yxyx() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x0101u)); }
[[nodiscard]] auto yxyy() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x1101u)); }
[[nodiscard]] auto yyxx() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x0011u)); }
[[nodiscard]] auto yyxy() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x1011u)); }
[[nodiscard]] auto yyyx() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x0111u)); }
[[nodiscard]] auto yyyy() const noexcept { return def<Vector<T, 4>>(detail::FunctionBuilder::current()->swizzle(Type::of<Vector<T, 4>>(), this->expression(), 4u, 0x1111u)); }

